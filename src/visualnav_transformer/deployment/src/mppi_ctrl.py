#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def yaw_to_quat_z_w(yaw: float):
    """2D yaw -> quaternion (x=y=0)."""
    half = 0.5 * yaw
    return math.sin(half), math.cos(half)  # z, w


class GnmToMppiPath(Node):
    def __init__(self):
        super().__init__('gnm_to_mppi_path')

        # Parameters
        self.declare_parameter('in_topic', '/chosen_trajectory')
        self.declare_parameter('out_topic', '/neupan_ref_traj_local')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('target_num_poses', 40)   # 你按 MPPI 期望改这个
        self.declare_parameter('min_dist_epsilon', 1e-6) # 防止重复点导致弧长为 0

        in_topic = self.get_parameter('in_topic').value
        out_topic = self.get_parameter('out_topic').value

        # QoS: input usually like sensor-ish; output keep last
        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.sub = self.create_subscription(Float32MultiArray, in_topic, self.cb, sub_qos)
        self.pub = self.create_publisher(Path, out_topic, pub_qos)

        self.get_logger().info(f"Sub: {in_topic}  -> Pub: {out_topic}")

    def cb(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 8 or (len(data) % 4) != 0:
            self.get_logger().warn(f"Unexpected data length: {len(data)} (need multiple of 4, >=8)")
            return

        frame_id = self.get_parameter('frame_id').value
        target_num = int(self.get_parameter('target_num_poses').value)
        eps = float(self.get_parameter('min_dist_epsilon').value)

        # Parse waypoints: [x, y, cos(yaw), sin(yaw)]
        arr = np.array(data, dtype=np.float64).reshape((-1, 4))
        x = arr[:, 0]
        y = arr[:, 1]
        cy = arr[:, 2]
        sy = arr[:, 3]
        yaw = np.arctan2(sy, cy)

        # Compute cumulative arc-length s
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.sqrt(dx * dx + dy * dy)

        # Remove (almost) duplicate consecutive points to avoid flat s
        keep = [0]
        for i in range(1, len(x)):
            if math.hypot(x[i] - x[keep[-1]], y[i] - y[keep[-1]]) > eps:
                keep.append(i)

        x = x[keep]
        y = y[keep]
        yaw = yaw[keep]

        if len(x) < 2:
            self.get_logger().warn("Not enough distinct waypoints after filtering.")
            return

        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.sqrt(dx * dx + dy * dy)
        s = np.concatenate(([0.0], np.cumsum(ds)))
        total = float(s[-1])

        # If all points same (total==0), just repeat
        if total <= eps:
            s_new = np.linspace(0.0, 0.0, target_num)
            x_new = np.full((target_num,), float(x[0]))
            y_new = np.full((target_num,), float(y[0]))
            yaw_unwrap = np.unwrap(yaw)
            yaw_new = np.full((target_num,), float(yaw_unwrap[0]))
        else:
            s_new = np.linspace(0.0, total, target_num)

            # x,y linear interpolation over s
            x_new = np.interp(s_new, s, x)
            y_new = np.interp(s_new, s, y)

            # yaw interpolation: unwrap then interp then wrap not strictly needed
            yaw_unwrap = np.unwrap(yaw)
            yaw_new = np.interp(s_new, s, yaw_unwrap)

        # Build Path
        out = Path()
        now = self.get_clock().now().to_msg()
        out.header.stamp = now
        out.header.frame_id = frame_id

        out.poses = []
        for i in range(target_num):
            ps = PoseStamped()
            ps.header.stamp = now
            ps.header.frame_id = frame_id
            ps.pose.position.x = float(x_new[i])
            ps.pose.position.y = float(y_new[i])
            ps.pose.position.z = 0.0
            qz, qw = yaw_to_quat_z_w(float(yaw_new[i]))
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)
            out.poses.append(ps)

        self.pub.publish(out)


def main():
    rclpy.init()
    node = GnmToMppiPath()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
