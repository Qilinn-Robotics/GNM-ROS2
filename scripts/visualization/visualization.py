import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time

class GNMVisualizer(Node):
    def __init__(self):
        super().__init__("gnm_visualizer")
        
        # QoS profile for better reliability
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/visualization_marker_array", qos_profile)
        self.annotated_image_pub = self.create_publisher(Image, "/gnm/annotated_image", qos_profile)
        
        # Subscribers
        self.waypoint_sub = self.create_subscription(
            Float32MultiArray, "/waypoint", self.waypoint_callback, qos_profile)
        self.chosen_traj_sub = self.create_subscription(
            Float32MultiArray, "/chosen_trajectory", self.chosen_traj_callback, qos_profile)
        self.sampled_actions_sub = self.create_subscription(
            Float32MultiArray, "/sampled_actions", self.sampled_actions_callback, qos_profile)
        
        # 自动获取图像话题
        image_topic = self._find_image_topic()
        self.get_logger().info(f"正在订阅图像话题: {image_topic}")
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile)
        
        self.bridge = CvBridge()
        self.current_image = None
        
        # 检测 OpenCV GUI 支持
        self.has_gui = self._check_gui_support()
        
        # 数据缓存
        self.last_waypoint = None
        self.last_chosen_traj = None
        self.last_sampled_actions = None
        self.last_update_time = self.get_clock().now()
        
        # Configuration
        self.frame_id = "base_link" # 机器人基座坐标系
        
        # 尝试从参数服务器获取轨迹长度，默认为 8
        self.declare_parameter("len_traj_pred", 8)
        self.len_traj_pred = self.get_parameter("len_traj_pred").value

        # Declare waypoint index parameter
        self.declare_parameter("waypoint_index", 2)
        
        # 用于限流日志的计时器
        self.last_log_time = time.time()
        
        self.get_logger().info(f"GNM Visualizer 启动成功！预测轨迹长度设为: {self.len_traj_pred}")
        if self.has_gui:
            self.get_logger().info("可视化模式: OpenCV 窗口直接显示 (包含投影轨迹和雷达图)")
        else:
            self.get_logger().warn("OpenCV GUI 不可用，将通过 ROS 话题发布图像")
            self.get_logger().info("  查看方式: ros2 run rqt_image_view rqt_image_view")
            self.get_logger().info("  话题名称: /gnm/annotated_image")
        self.get_logger().info("同时保留 RViz MarkerArray 订阅 /visualization_marker_array 话题")

    def _check_gui_support(self):
        """检测 OpenCV 是否支持 GUI 显示"""
        try:
            # 尝试创建一个测试窗口
            test_img = np.zeros((10, 10, 3), dtype=np.uint8)
            cv2.imshow("__test__", test_img)
            cv2.destroyWindow("__test__")
            cv2.waitKey(1)
            return True
        except:
            return False

    def _find_image_topic(self):
        """自动寻找包含 'image' 或 'camera' 的 Image 类型话题"""
        # 等待话题列表可用
        time.sleep(1.0)
        topic_list = self.get_topic_names_and_types()
        
        # 优先寻找包含 'front' 的
        for topic, types in topic_list:
            if 'sensor_msgs/msg/Image' in types and 'front' in topic:
                return topic
        # 其次寻找包含 'camera' 或 'image' 的
        for topic, types in topic_list:
            if 'sensor_msgs/msg/Image' in types and ('camera' in topic or 'image' in topic):
                return topic
        # 默认值
        return "/front/camera_image"
        
    def chosen_traj_callback(self, msg):
        """Callback for the chosen trajectory"""
        data = np.array(msg.data)
        if len(data) == 0:
            return
            
        # Try to infer dimension and length
        possible_traj_lens = [5, 8]
        possible_dims = [4, 2]
        
        found = False
        # First check if current config works
        current_len = self.len_traj_pred
        for d in possible_dims:
            if len(data) == current_len * d:
                self.last_chosen_traj = data.reshape(current_len, d)
                found = True
                break
        
        # If not, search for other configs
        if not found:
            for tl in possible_traj_lens:
                for d in possible_dims:
                    if len(data) == tl * d:
                        self.len_traj_pred = tl # Update global config
                        self.last_chosen_traj = data.reshape(tl, d)
                        found = True
                        self.get_logger().info(f"Auto-detected chosen trajectory shape: {tl}x{d}", once=True)
                        break
                if found: break
                
        # Fallback
        if not found:
             if len(data) % 2 == 0:
                 self.last_chosen_traj = data.reshape(-1, 2)
                 
    def waypoint_callback(self, msg):
        """可视化选中的路标点 (Chosen Waypoint)"""
        if len(msg.data) < 2:
            return
            
        # 存入缓存供图像显示
        self.last_waypoint = np.array(msg.data)
        self.last_update_time = self.get_clock().now()

        marker_array = MarkerArray()
        
        # 先删除之前的路标点
        delete_marker = Marker()
        delete_marker.ns = "chosen_waypoint"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        
        # 判断数据格式
        # 如果是 4 个值，通常是 [x, y, cos(yaw), sin(yaw)]
        # 如果是 2 个值，是 [x, y]
        # 如果是更多且为偶数，视为多个 [x, y] 点
        if len(msg.data) == 4:
            num_pts = 1
            is_with_orientation = True
        elif len(msg.data) == 2:
            num_pts = 1
            is_with_orientation = False
        else:
            # 假设是多个 (x, y) 点
            num_pts = len(msg.data) // 2
            is_with_orientation = False

        for i in range(num_pts):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "chosen_waypoint"
            marker.id = i
            marker.type = Marker.ARROW if is_with_orientation else Marker.SPHERE
            marker.action = Marker.ADD
            
            # 坐标转换
            marker.pose.position.x = msg.data[2*i]
            marker.pose.position.y = msg.data[2*i+1]
            marker.pose.position.z = 0.2 # 稍微悬浮一点
            
            if is_with_orientation:
                # 计算四元数 (从 yaw 转换)
                # 对于单个点，索引是 2, 3
                cos_yaw = msg.data[2]
                sin_yaw = msg.data[3]
                yaw = np.arctan2(sin_yaw, cos_yaw)
                marker.pose.orientation.z = np.sin(yaw / 2.0)
                marker.pose.orientation.w = np.cos(yaw / 2.0)
                
                # 箭头大小
                marker.scale.x = 0.3  # 长度
                marker.scale.y = 0.05 # 箭头宽度
                marker.scale.z = 0.05 # 箭头高度
            else:
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.pose.orientation.w = 1.0
            
            marker.color.a = 1.0
            marker.color.r = 1.0 # 红色表示选中的路径
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

    def sampled_actions_callback(self, msg):
        """可视化所有采样的轨迹 (Sampled Trajectories)"""
        # msg.data[0] 是标志位，后面是打平的轨迹数据
        data = np.array(msg.data[1:])
        
        if len(data) == 0:
            return

        # ROS2 throttle logging
        current_time = time.time()
        if current_time - self.last_log_time > 5.0:
            self.get_logger().info(f"Received sampled actions data length: {len(data)}")
            self.last_log_time = current_time

        # 自动推断轨迹长度和维度
        possible_traj_lens = [5, 8]
        possible_dims = [4, 2]
        
        found_config = False
        dim = 2
        for tl in possible_traj_lens:
            for d in possible_dims:
                if len(data) % (tl * d) == 0:
                    self.len_traj_pred = tl
                    dim = d
                    found_config = True
                    break
            if found_config:
                break
        
        num_samples = len(data) // (self.len_traj_pred * dim)
        if num_samples == 0:
            return
            
        try:
            trajectories = data.reshape(num_samples, self.len_traj_pred, dim)
            # 存入缓存供图像显示
            self.last_sampled_actions = trajectories
        except ValueError as e:
            self.get_logger().warn(f"无法对齐轨迹数据: {e}")
            return
        
        marker_array = MarkerArray()
        
        # 先删除之前的轨迹
        delete_marker = Marker()
        delete_marker.ns = "sampled_trajectories"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub.publish(marker_array)
        
        marker_array = MarkerArray()
        for i in range(num_samples):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "sampled_trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            marker.scale.x = 0.02 # 线宽
            
            # 蓝色半透明表示采样路径
            marker.color.a = 0.4
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            
            # 添加轨迹点
            # 假设轨迹是从 (0,0) 开始的相对坐标
            start_p = Point()
            start_p.x = 0.0
            start_p.y = 0.0
            start_p.z = 0.0
            marker.points.append(start_p)
            
            for j in range(self.len_traj_pred):
                p = Point()
                p.x = float(trajectories[i, j, 0])
                p.y = float(trajectories[i, j, 1])
                p.z = 0.0
                marker.points.append(p)
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

    def image_callback(self, msg):
        """使用 OpenCV 在图像上直接绘制路标点和轨迹"""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            h, w = img.shape[:2]
            
            # 创建透明叠加层用于绘制采样轨迹 (蓝线)
            overlay = img.copy()
            
            # 1. 绘制采样轨迹 (候选 Waypoints - 蓝色)
            if self.last_sampled_actions is not None:
                for i, traj in enumerate(self.last_sampled_actions):
                    points = []
                    u0, v0 = self._project(0, 0, w, h)
                    
                    for pt in traj:
                        u, v = self._project(pt[0], pt[1], w, h)
                        cv2.circle(overlay, (u, v), 2, (255, 100, 0), -1)
                        points.append((u, v))
                    
                    last_p = (u0, v0)
                    for p in points:
                        cv2.line(overlay, last_p, p, (255, 100, 0), 2)
                        last_p = p
            
            # 应用透明度: 40% 蓝线 + 60% 原图
            alpha = 0.4
            cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)
            
            # 1.5 绘制选中的轨迹 (黄色 - 粗线)
            if self.last_chosen_traj is not None:
                points = []
                u0, v0 = self._project(0, 0, w, h)
                for pt in self.last_chosen_traj:
                    u, v = self._project(pt[0], pt[1], w, h)
                    cv2.circle(img, (u, v), 4, (0, 255, 255), -1)
                    points.append((u, v))
                
                last_p = (u0, v0)
                for p in points:
                    cv2.line(img, last_p, p, (0, 255, 255), 4)
                    last_p = p

            # 2. 绘制从 /waypoint 话题接收的目标点 (洋红色大星星)
            if self.last_waypoint is not None and len(self.last_waypoint) >= 2:
                wp_u, wp_v = self._project(self.last_waypoint[0], self.last_waypoint[1], w, h)
                # 绘制大星星标记
                cv2.drawMarker(img, (wp_u, wp_v), (255, 0, 255), cv2.MARKER_STAR, 30, 3)
                cv2.putText(img, "Goal", (wp_u + 15, wp_v - 15), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            # 3. 绘制选中轨迹上的目标点 (红色)
            # 自动匹配逻辑：在选中的轨迹中寻找与 /waypoint 最近的点作为索引
            waypoint_idx = -1
            if self.last_chosen_traj is not None and self.last_waypoint is not None:
                min_dist = float('inf')
                for i, pt in enumerate(self.last_chosen_traj):
                    # 计算距离
                    dist = (pt[0] - self.last_waypoint[0])**2 + (pt[1] - self.last_waypoint[1])**2
                    if dist < min_dist:
                        min_dist = dist
                        waypoint_idx = i
                
                # 如果找不到匹配点（距离过大），则回退到默认参数或跳过
                if min_dist > 0.1: # 允许一定的误差
                     try:
                        waypoint_idx = self.get_parameter("waypoint_index").value
                     except:
                        waypoint_idx = 2
            else:
                 try:
                    waypoint_idx = self.get_parameter("waypoint_index").value
                 except:
                    waypoint_idx = 2
            
            if self.last_chosen_traj is not None and len(self.last_chosen_traj) > waypoint_idx and waypoint_idx >= 0:
                wpt = self.last_chosen_traj[waypoint_idx]
                u, v = self._project(wpt[0], wpt[1], w, h)
                cv2.circle(img, (u, v), 12, (0, 0, 255), -1) # 红色大圆点
                cv2.circle(img, (u, v), 14, (255, 255, 255), 2) # 白色外圈
                
                # 在红点上写上序号数字
                font = cv2.FONT_HERSHEY_SIMPLEX
                text = str(waypoint_idx)
                text_size = cv2.getTextSize(text, font, 0.5, 2)[0]
                text_x = u - text_size[0] // 2
                text_y = v + text_size[1] // 2
                cv2.putText(img, text, (text_x, text_y), font, 0.5, (255, 255, 255), 2)
                
                if len(wpt) >= 4:
                    cos_yaw, sin_yaw = wpt[2], wpt[3]
                    # 计算物理上的终点
                    u2, v2 = self._project(wpt[0] + cos_yaw*0.5, 
                                         wpt[1] + sin_yaw*0.5, w, h)
                    
                    # 计算图像空间的方向向量
                    dx = u2 - u
                    dy = v2 - v
                    norm = np.sqrt(dx*dx + dy*dy)
                    
                    # 如果能计算出方向，则使用固定长度画箭头
                    if norm > 1e-3:
                        arrow_len = 25 # 像素
                        u_end = int(u + (dx/norm) * arrow_len)
                        v_end = int(v + (dy/norm) * arrow_len)
                        cv2.arrowedLine(img, (u, v), (u_end, v_end), (0, 0, 255), 3, tipLength=0.3)
                    else:
                        # 只有当原地不动或无法计算方向时，才回退到投影点（或者不画）
                        pass

            # 4. 添加图例 (Legend)
            legend_x, legend_y = 20, 30
            # 背景板
            cv2.rectangle(img, (legend_x-10, legend_y-20), (legend_x+220, legend_y+90), (0,0,0), -1)
            cv2.rectangle(img, (legend_x-10, legend_y-20), (legend_x+220, legend_y+90), (150,150,150), 1)
            
            # 候选轨迹
            cv2.line(img, (legend_x, legend_y), (legend_x+30, legend_y), (255, 100, 0), 2)
            cv2.putText(img, "Candidate Paths", (legend_x+40, legend_y+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 选中轨迹
            cv2.line(img, (legend_x, legend_y+25), (legend_x+30, legend_y+25), (0, 255, 255), 3)
            cv2.putText(img, "Chosen Path", (legend_x+40, legend_y+30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 最终目标点
            cv2.drawMarker(img, (legend_x+15, legend_y+50), (255, 0, 255), cv2.MARKER_STAR, 15, 2)
            cv2.putText(img, "Goal Waypoint", (legend_x+40, legend_y+55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # 轨迹目标点
            cv2.circle(img, (legend_x+15, legend_y+75), 6, (0, 0, 255), -1)
            cv2.putText(img, "Traj Target", (legend_x+40, legend_y+80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # 3. 绘制雷达图 (右上角)
            self._draw_radar(img)

            # 尝试直接显示（如果支持 GUI）
            if self.has_gui:
                try:
                    cv2.imshow("GNM Observation (Annotated)", img)
                    cv2.waitKey(1)
                except:
                    self.has_gui = False  # GUI 失效，切换到发布模式
                    self.get_logger().warn("OpenCV 窗口显示失败，切换到话题发布模式")
            
            # 同时发布 ROS 话题（总是执行，方便调试）
            try:
                annotated_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                annotated_msg.header.stamp = msg.header.stamp
                annotated_msg.header.frame_id = msg.header.frame_id
                self.annotated_image_pub.publish(annotated_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    def _project(self, x, y, w, h):
        """简单的透视投影，将机器人坐标系 (x-前, y-左) 映射到图像像素"""
        # 这是一个启发式投影，假设相机安装在约 0.4m 高度，略微下倾
        cam_h = 0.4
        cam_x_off = 0.1
        # 焦距近似 (对于 160 宽度的图像，焦距约等于宽度)
        f = w 
        # 限制 x 最小值防止除零
        x_eff = max(x + cam_x_off, 0.1)
        
        # u: 水平方向 (y 是左正，图像右是正)
        u = w/2 - (y / x_eff) * f
        # v: 垂直方向 (x 越大越靠上)
        v = h/2 + (cam_h / x_eff) * f
        
        return int(np.clip(u, -w, 2*w)), int(np.clip(v, -h, 2*h))

    def _draw_radar(self, img):
        """在图像右上角绘制一个小雷达图 (俯视图)"""
        h, w = img.shape[:2]
        radar_size = 120
        if h < radar_size or w < radar_size:
            return
            
        radar_img = np.zeros((radar_size, radar_size, 3), dtype=np.uint8)
        cx, cy = radar_size // 2, radar_size - 15
        scale = 20 # 1m = 20px
        
        # 画网格
        for r in range(1, 4):
            cv2.circle(radar_img, (cx, cy), r * scale, (70, 70, 70), 1)
            cv2.putText(radar_img, f"{r}m", (cx + r*scale - 10, cy + 12), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (100, 100, 100), 1)
            
        # 画采样轨迹
        if self.last_sampled_actions is not None:
            for i, traj in enumerate(self.last_sampled_actions):
                color = (255, 100, 0)
                thickness = 1
                last_p = (cx, cy)
                for pt in traj:
                    px = int(cx - pt[1] * scale)
                    py = int(cy - pt[0] * scale)
                    cv2.line(radar_img, last_p, (px, py), color, thickness)
                    last_p = (px, py)
        
        # 画选中的轨迹
        if self.last_chosen_traj is not None:
            color = (0, 255, 255)
            thickness = 2
            last_p = (cx, cy)
            for pt in self.last_chosen_traj:
                px = int(cx - pt[1] * scale)
                py = int(cy - pt[0] * scale)
                cv2.line(radar_img, last_p, (px, py), color, thickness)
                last_p = (px, py)
                    
        # 画路标点
        if self.last_waypoint is not None:
            px = int(cx - self.last_waypoint[1] * scale)
            py = int(cy - self.last_waypoint[0] * scale)
            cv2.circle(radar_img, (px, py), 5, (0, 0, 255), -1)
            
        # 叠加到主图右上角 (带透明度)
        roi = img[10:10+radar_size, w-radar_size-10:w-10]
        cv2.addWeighted(roi, 0.5, radar_img, 0.5, 0, roi)

def main(args=None):
    rclpy.init(args=args)
    visualizer = None
    try:
        visualizer = GNMVisualizer()
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        if visualizer is not None:
            if visualizer.has_gui:
                try:
                    cv2.destroyAllWindows()
                except:
                    pass
            visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
