import argparse
import os
import time

import numpy as np
import rclpy
import torch
import yaml
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from PIL import Image as PILImage

# ROS
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool

# UTILS
from visualnav_transformer.deployment.src.topic_names import (
    IMAGE_TOPIC,
    SAMPLED_ACTIONS_TOPIC,
    WAYPOINT_TOPIC,
    REACHED_GOAL_TOPIC,
    CHOSEN_TRAJECTORY_TOPIC,
)
from visualnav_transformer.deployment.src.utils import (
    load_model,
    msg_to_pil,
    to_numpy,
    transform_images,
)
from visualnav_transformer.train.vint_train.training.train_utils import get_action

# CONSTANTS
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_WEIGHTS_PATH = os.path.join(SCRIPT_DIR, "model_weights")
ROBOT_CONFIG_PATH = os.path.join(SCRIPT_DIR, "config/robot.yaml")
MODEL_CONFIG_PATH = os.path.join(SCRIPT_DIR, "config/models.yaml")
TOPOMAP_IMAGES_DIR = os.path.join(SCRIPT_DIR, "../topomaps/images")

# Load robot configuration with better error handling
try:
    with open(ROBOT_CONFIG_PATH, "r") as f:
        robot_config = yaml.safe_load(f)
    MAX_V = robot_config["max_v"]
    MAX_W = robot_config["max_w"]
    RATE = robot_config["frame_rate"]
except FileNotFoundError:
    raise FileNotFoundError(
        f"Robot config file not found at '{ROBOT_CONFIG_PATH}'. "
        f"Make sure you're running this script from the deployment/src directory "
        f"or that the config path is correct."
    )
except KeyError as e:
    raise KeyError(f"Missing required key {e} in robot config file '{ROBOT_CONFIG_PATH}'")


class NavigationNode(Node):
    def __init__(self, args, model, model_params, topomap, device, noise_scheduler=None, num_diffusion_iters=None):
        super().__init__("navigation_node")
        
        # Declare and set ROS parameters
        self.declare_parameter("waypoint_index", args.waypoint)
        
        # Store parameters
        self.args = args
        self.model = model
        self.model_params = model_params
        self.topomap = topomap
        self.device = device
        self.noise_scheduler = noise_scheduler
        self.num_diffusion_iters = num_diffusion_iters
        self.RATE = RATE
        self.MAX_V = MAX_V
        
        # Initialize state variables
        self.context_queue = []
        self.context_size = model_params["context_size"]
        self.closest_node = 0
        
        # Goal node setup
        assert -1 <= args.goal_node < len(topomap), "Invalid goal index"
        if args.goal_node == -1:
            self.goal_node = len(topomap) - 1
        else:
            self.goal_node = args.goal_node
        self.reached_goal = False
        
        # Create callback group for concurrent execution
        self.cb_group = ReentrantCallbackGroup()
        
        # Create subscription with callback group
        self.create_subscription(
            Image, 
            IMAGE_TOPIC, 
            self.callback_obs, 
            10, 
            callback_group=self.cb_group
        )
        
        # Create publishers
        self.waypoint_pub = self.create_publisher(Float32MultiArray, WAYPOINT_TOPIC, 1)
        self.sampled_actions_pub = self.create_publisher(
            Float32MultiArray, SAMPLED_ACTIONS_TOPIC, 1
        )
        self.chosen_traj_pub = self.create_publisher(Float32MultiArray, CHOSEN_TRAJECTORY_TOPIC, 1)
        self.reached_goal_pub = self.create_publisher(Bool, REACHED_GOAL_TOPIC, 1)
        
        # Create timer for navigation loop
        self.create_timer(
            1.0 / self.RATE, 
            self.timer_callback, 
            callback_group=self.cb_group
        )
    
    def callback_obs(self, msg):
        """Callback for receiving observation images"""
        obs_img = msg_to_pil(msg)
        if len(self.context_queue) < self.context_size + 1:
            self.context_queue.append(obs_img)
        else:
            self.context_queue.pop(0)
            self.context_queue.append(obs_img)
    
    def timer_callback(self):
        """Main navigation inference loop triggered by timer"""
        self.reached_goal_pub.publish(Bool(data=bool(self.reached_goal)))
        if self.reached_goal:
            print("Goal already reached, skipping inference.")
            return
        
        # Check if we have enough context
        if len(self.context_queue) <= self.model_params["context_size"]:
            return
        
        waypoint_msg = Float32MultiArray()
        
        if self.model_params["model_type"] == "nomad":
            obs_images = transform_images(
                self.context_queue, self.model_params["image_size"], center_crop=False
            )
            obs_images = torch.split(obs_images, 3, dim=1)
            obs_images = torch.cat(obs_images, dim=1)
            obs_images = obs_images.to(self.device)
            mask = torch.zeros(1).long().to(self.device)

            start = max(self.closest_node - self.args.radius, 0)
            end = min(self.closest_node + self.args.radius + 1, self.goal_node)
            goal_image = [
                transform_images(
                    g_img, self.model_params["image_size"], center_crop=False
                ).to(self.device)
                for g_img in self.topomap[start : end + 1]
            ]
            goal_image = torch.concat(goal_image, dim=0)

            obsgoal_cond = self.model(
                "vision_encoder",
                obs_img=obs_images.repeat(len(goal_image), 1, 1, 1),
                goal_img=goal_image,
                input_goal_mask=mask.repeat(len(goal_image)),
            )
            dists = self.model("dist_pred_net", obsgoal_cond=obsgoal_cond)
            dists = to_numpy(dists.flatten())
            min_idx = np.argmin(dists)
            self.closest_node = min_idx + start
            print(f"closest node: {self.closest_node}, distance: {dists[min_idx]}")
            sg_idx = min(
                min_idx + int(dists[min_idx] < self.args.close_threshold),
                len(obsgoal_cond) - 1,
            )
            obs_cond = obsgoal_cond[sg_idx].unsqueeze(0)

            # infer action
            if self.noise_scheduler is not None:
                with torch.no_grad():
                    # encoder vision features
                    if len(obs_cond.shape) == 2:
                        obs_cond = obs_cond.repeat(self.args.num_samples, 1)
                    else:
                        obs_cond = obs_cond.repeat(self.args.num_samples, 1, 1)

                    # initialize action from Gaussian noise
                    noisy_action = torch.randn(
                        (self.args.num_samples, self.model_params["len_traj_pred"], 2),
                        device=self.device,
                    )
                    naction = noisy_action

                    # init scheduler
                    self.noise_scheduler.set_timesteps(self.num_diffusion_iters)

                    for k in self.noise_scheduler.timesteps[:]:
                        # predict noise
                        noise_pred = self.model(
                            "noise_pred_net",
                            sample=naction,
                            timestep=k,
                            global_cond=obs_cond,
                        )
                        # inverse diffusion step (remove noise)
                        naction = self.noise_scheduler.step(
                            model_output=noise_pred, timestep=k, sample=naction
                        ).prev_sample

                naction = to_numpy(get_action(naction))
                sampled_actions_msg = Float32MultiArray()
                sampled_actions_msg.data = np.concatenate(
                    (np.array([0]), naction.flatten())
                ).tolist()
                self.sampled_actions_pub.publish(sampled_actions_msg)
                
                # Publish chosen trajectory (the first sample)
                chosen_traj_msg = Float32MultiArray()
                chosen_traj_msg.data = naction[0].flatten().tolist()
                self.chosen_traj_pub.publish(chosen_traj_msg)

                naction = naction[0]
                chosen_waypoint = naction[self.args.waypoint]

                if self.model_params["normalize"]:
                    chosen_waypoint *= self.MAX_V / self.RATE
                waypoint_msg.data = chosen_waypoint.tolist()
                self.waypoint_pub.publish(waypoint_msg)
        else:
            # === GNM / ViNT model logic ===
            start = max(self.closest_node - self.args.radius, 0)
            end = min(self.closest_node + self.args.radius + 1, self.goal_node)
            
            batch_obs_imgs = []
            batch_goal_data = []
            for i, sg_img in enumerate(self.topomap[start: end + 1]):
                transf_obs_img = transform_images(self.context_queue, self.model_params["image_size"])
                goal_data = transform_images(sg_img, self.model_params["image_size"])
                batch_obs_imgs.append(transf_obs_img)
                batch_goal_data.append(goal_data)
                
            batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(self.device)
            batch_goal_data = torch.cat(batch_goal_data, dim=0).to(self.device)

            # GNM/ViNT directly output distances and waypoints, no diffusion needed
            with torch.no_grad():
                distances, waypoints = self.model(batch_obs_imgs, batch_goal_data)
            distances = to_numpy(distances)
            waypoints = to_numpy(waypoints)
            
            min_dist_idx = np.argmin(distances)
            
            # Publish visualization
            sampled_actions_msg = Float32MultiArray()
            sampled_actions_msg.data = np.concatenate((np.array([0]), waypoints.flatten())).tolist()
            self.sampled_actions_pub.publish(sampled_actions_msg)

            # Select best waypoint
            if distances[min_dist_idx] > self.args.close_threshold:
                chosen_traj_idx = min_dist_idx
                chosen_waypoint = waypoints[chosen_traj_idx][self.args.waypoint]
                self.closest_node = start + min_dist_idx
            else:
                chosen_traj_idx = min(min_dist_idx + 1, len(waypoints) - 1)
                chosen_waypoint = waypoints[chosen_traj_idx][self.args.waypoint]
                self.closest_node = min(start + min_dist_idx + 1, self.goal_node)
            
            # Publish chosen trajectory
            chosen_traj_msg = Float32MultiArray()
            chosen_traj_msg.data = waypoints[chosen_traj_idx].flatten().tolist()
            self.chosen_traj_pub.publish(chosen_traj_msg)
            
            # Normalize
            if self.model_params["normalize"]:
                chosen_waypoint[:2] *= (self.MAX_V / self.RATE)
                
            waypoint_msg.data = chosen_waypoint.tolist()
            self.waypoint_pub.publish(waypoint_msg)
            
            print(f"Closest node: {self.closest_node}/{self.goal_node} | Distance: {distances[min_dist_idx].item():.2f}")
        
        # Check if goal is reached
        self.reached_goal = self.closest_node == self.goal_node
        if self.reached_goal:
            print("Reached goal! Stopping...")


def main(args: argparse.Namespace, device: torch.device):
    # load model parameters
    try:
        with open(MODEL_CONFIG_PATH, "r") as f:
            model_paths = yaml.safe_load(f)
    except FileNotFoundError:
        raise FileNotFoundError(
            f"Model config file not found at '{MODEL_CONFIG_PATH}'. "
            f"Make sure you're running this script from the correct directory (deployment/src)."
        )

    if args.model not in model_paths:
        raise ValueError(
            f"Model '{args.model}' not found in config. Available models: {list(model_paths.keys())}"
        )
    
    model_config_path = model_paths[args.model]["config_path"]
    try:
        with open(model_config_path, "r") as f:
            model_params = yaml.safe_load(f)
    except FileNotFoundError:
        raise FileNotFoundError(
            f"Model parameter file not found at '{model_config_path}'"
        )

    # load model weights
    ckpth_path = model_paths[args.model]["ckpt_path"]
    if os.path.exists(ckpth_path):
        print(f"Loading model from {ckpth_path}")
    else:
        raise FileNotFoundError(f"Model weights not found at {ckpth_path}")
    model = load_model(
        ckpth_path,
        model_params,
        device,
    )
    model = model.to(device)
    model.eval()

    # Initialize diffusion scheduler only for nomad model
    noise_scheduler = None
    num_diffusion_iters = None
    if model_params["model_type"] == "nomad":
        num_diffusion_iters = model_params["num_diffusion_iters"]
        noise_scheduler = DDPMScheduler(
            num_train_timesteps=model_params["num_diffusion_iters"],
            beta_schedule="squaredcos_cap_v2",
            clip_sample=True,
            prediction_type="epsilon",
        )

    # load topomap
    topomap_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
    
    if not os.path.exists(topomap_dir):
        raise FileNotFoundError(
            f"Topomap directory not found at '{topomap_dir}'. "
            f"Make sure the topomap has been created and the path is correct. "
            f"Current working directory: {os.getcwd()}"
        )
    
    topomap_filenames = sorted(
        os.listdir(topomap_dir),
        key=lambda x: int(x.split(".")[0]),
    )
    
    if len(topomap_filenames) == 0:
        raise ValueError(
            f"No images found in topomap directory '{topomap_dir}'. "
            f"Please create a topomap first using create_topomap.py"
        )
    
    num_nodes = len(topomap_filenames)
    print(f"Loading topomap with {num_nodes} nodes from '{topomap_dir}'")
    
    topomap = []
    for i in range(num_nodes):
        image_path = os.path.join(topomap_dir, topomap_filenames[i])
        try:
            topomap.append(PILImage.open(image_path))
        except Exception as e:
            raise IOError(f"Failed to load image '{image_path}': {e}")

    print("------------------------------------------------")
    print("Navigation Parameters:")
    print(f"  Model: {args.model}")
    print(f"  Device: {device}")
    print(f"  Robot Config: {ROBOT_CONFIG_PATH}")
    print(f"  Max Linear Velocity (MAX_V): {MAX_V} m/s")
    print(f"  Max Angular Velocity (MAX_W): {MAX_W} rad/s")
    print(f"  Frame Rate (Planning Frequency): {RATE} Hz")
    print(f"  Waypoint Index: {args.waypoint}")
    print(f"  Goal Node Index: {args.goal_node}")
    print(f"  Topomap Directory: {args.dir}")
    print(f"  Close Threshold: {args.close_threshold}")
    print(f"  Radius: {args.radius}")
    print(f"  Normalize: {model_params.get('normalize', 'N/A')}")
    print("------------------------------------------------")

    # ROS
    rclpy.init()
    
    try:
        # Create node with all necessary parameters
        node = NavigationNode(
            args=args,
            model=model,
            model_params=model_params,
            topomap=topomap,
            device=device,
            noise_scheduler=noise_scheduler,
            num_diffusion_iters=num_diffusion_iters
        )
        
        # Create multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            # Spin the executor
            executor.spin()
        except KeyboardInterrupt:
            print("Keyboard interrupt, shutting down...")
        finally:
            executor.shutdown()
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    # Load defaults from YAML config
    BRL_CONFIG_PATH = os.path.join(SCRIPT_DIR, "config/gnm_config_for_brlrobot.yaml")
    brl_defaults = {}
    if os.path.exists(BRL_CONFIG_PATH):
        try:
            with open(BRL_CONFIG_PATH, "r") as f:
                brl_defaults = yaml.safe_load(f) or {}
            print(f"Loaded configuration from {BRL_CONFIG_PATH}")
        except Exception as e:
            print(f"Warning: Failed to load config from {BRL_CONFIG_PATH}: {e}")
    else:
        print(f"Warning: Config file {BRL_CONFIG_PATH} not found. Using hardcoded defaults.")

    parser = argparse.ArgumentParser(
        description="Code to run GNM DIFFUSION EXPLORATION on the locobot"
    )
    parser.add_argument(
        "--model",
        "-m",
        default=brl_defaults.get("model", "nomad"),
        type=str,
        help="model name (only nomad is supported) (hint: check config/models.yaml) (default: nomad)",
    )
    parser.add_argument(
        "--waypoint",
        "-w",
        default=brl_defaults.get("waypoint", 2),  # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or
        how many waypoints your model predicts) (default: 2)""",
    )
    parser.add_argument(
        "--dir",
        "-d",
        default=brl_defaults.get("dir", "topomap"),
        type=str,
        help="path to topomap images",
    )
    parser.add_argument(
        "--goal-node",
        "-g",
        default=brl_defaults.get("goal_node", -1),
        type=int,
        help="""goal node index in the topomap (if -1, then the goal node is
        the last node in the topomap) (default: -1)""",
    )
    parser.add_argument(
        "--close-threshold",
        "-t",
        default=brl_defaults.get("close_threshold", 3),
        type=int,
        help="""temporal distance within the next node in the topomap before
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--radius",
        "-r",
        default=brl_defaults.get("radius", 4),
        type=int,
        help="""temporal number of locobal nodes to look at in the topopmap for
        localization (default: 2)""",
    )
    parser.add_argument(
        "--num-samples",
        "-n",
        default=brl_defaults.get("num_samples", 8),
        type=int,
        help=f"Number of actions sampled from the exploration model (default: 8)",
    )
    parser.add_argument(
        "--device",
        "-dev",
        default=brl_defaults.get("device", "cuda"),
        type=str,
        help="device to run the model on (default: cuda)",
    )
    args = parser.parse_args()
    
    if args.device == "cuda" and not torch.cuda.is_available():
        print("CUDA not available, using CPU")
        device = torch.device("cpu")
    else:
        device = torch.device(args.device)
        
    print(f"Using {device}")
    main(args, device)
