from typing import Tuple
import os
import numpy as np

# ROS
import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import Twist
from ros_data import ROSData
from std_msgs.msg import Bool, Float32MultiArray
from topic_names import REACHED_GOAL_TOPIC, WAYPOINT_TOPIC
from utils import clip_angle

# CONSTS
# Use absolute path relative to this script
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_DIR, "config/robot.yaml")
# Try to load the BRL config for PID gains
BRL_CONFIG_PATH = os.path.join(SCRIPT_DIR, "config/gnm_config_for_brlrobot.yaml")

with open(CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)

# Default gains
KP_V = robot_config.get("kp_v", 1.0)
KP_W = robot_config.get("kp_w", 0.7)

if os.path.exists(BRL_CONFIG_PATH):
    try:
        with open(BRL_CONFIG_PATH, "r") as f:
            brl_config = yaml.safe_load(f)
            if brl_config:
                KP_V = brl_config.get("kp_v", KP_V)
                KP_W = brl_config.get("kp_w", KP_W)
        print(f"Loaded PID gains from {BRL_CONFIG_PATH}")
    except Exception as e:
        print(f"Warning: Failed to load config from {BRL_CONFIG_PATH}: {e}")

MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC = robot_config["vel_navi_topic"]
DT = 1 / robot_config["frame_rate"]
RATE = robot_config.get("control_frequency", 9)
EPS = 1e-8
WAYPOINT_TIMEOUT = 1  # seconds # TODO: tune this
FLIP_ANG_VEL = np.pi / 4

def clip_angle(theta) -> float:
    """Clip angle to [-pi, pi]"""
    theta %= 2 * np.pi
    if -np.pi < theta < np.pi:
        return theta
    return theta - 2 * np.pi


def pd_controller(waypoint: np.ndarray) -> Tuple[float]:
    """PD controller for the robot"""
    assert (
        len(waypoint) == 2 or len(waypoint) == 4
    ), "waypoint must be a 2D or 4D vector"
    if len(waypoint) == 2:
        dx, dy = waypoint
    else:
        dx, dy, hx, hy = waypoint
    # this controller only uses the predicted heading if dx and dy near zero
    if len(waypoint) == 4 and np.abs(dx) < EPS and np.abs(dy) < EPS:
        v = 0
        w = clip_angle(np.arctan2(hy, hx)) / DT
    elif np.abs(dx) < EPS:
        v = 0
        w = np.sign(dy) * np.pi / (2 * DT)
    else:
        v = dx / DT
        w = np.arctan(dy / dx) / DT
    
    # Apply gains
    v *= KP_V
    w *= KP_W

    v = np.clip(v, 0, MAX_V)
    w = np.clip(w, -MAX_W, MAX_W)
    return v, w


class PDController(Node):
    def __init__(self):
        super().__init__("pd_controller")
        
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, WAYPOINT_TOPIC, self.callback_drive, 1
        )
        self.reached_goal_sub = self.create_subscription(
            Bool, REACHED_GOAL_TOPIC, self.callback_reached_goal, 1
        )
        self.vel_out = self.create_publisher(Twist, VEL_TOPIC, 1)
        
        self.waypoint = ROSData(WAYPOINT_TIMEOUT, name="waypoint", clock=self.get_clock())
        self.reached_goal = False
        self.reverse_mode = False
        self.vel_msg = Twist()
        
        self.timer = self.create_timer(1.0 / RATE, self.control_loop)
        
        self.get_logger().info("------------------------------------------------")
        self.get_logger().info("PD Controller Configuration:")
        self.get_logger().info(f"  Control Rate (RATE): {RATE} Hz")
        self.get_logger().info(f"  Frame Rate (Planned): {robot_config['frame_rate']} Hz")
        self.get_logger().info(f"  Time Step (DT): {DT:.4f} s")
        self.get_logger().info(f"  Max V: {MAX_V} m/s")
        self.get_logger().info(f"  Max W: {MAX_W} rad/s")
        self.get_logger().info(f"  KP_V: {KP_V}")
        self.get_logger().info(f"  KP_W: {KP_W}")
        self.get_logger().info(f"  Vel Topic: {VEL_TOPIC}")
        self.get_logger().info("------------------------------------------------")
        self.get_logger().info("Registered with master node. Waiting for waypoints...")

    def callback_drive(self, waypoint_msg: Float32MultiArray):
        """Callback function for the waypoint subscriber"""
        # self.get_logger().info("setting waypoint")
        self.waypoint.set(waypoint_msg.data)

    def callback_reached_goal(self, reached_goal_msg: Bool):
        """Callback function for the reached goal subscriber"""
        self.reached_goal = reached_goal_msg.data

    def control_loop(self):
        self.vel_msg = Twist()
        if self.reached_goal:
            self.vel_out.publish(self.vel_msg)
            self.get_logger().info("Reached goal! Stopping...")
            # Ideally we might want to stop the node or just keep publishing 0
            return
        elif self.waypoint.is_valid(verbose=True):
            v, w = pd_controller(self.waypoint.get())
            if self.reverse_mode:
                v *= -1
            self.vel_msg.linear.x = float(v)
            self.vel_msg.angular.z = float(w)
            self.get_logger().info(f"publishing new vel: {v:.2f}, {w:.2f}")
        
        self.vel_out.publish(self.vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
