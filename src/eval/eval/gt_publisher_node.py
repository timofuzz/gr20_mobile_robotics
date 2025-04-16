#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

def load_ground_truth_tum(path, time_offset=0.0):
    """
    Loads ground truth poses from a TUM-style file.
    Each line: seconds x y z qx qy qz qw
    Returns a list of (timestamp, position, quaternion), with positions offset so the first pose after time offset is at (0,0,0).
    """
    gt_poses = []
    with open(path, 'r') as f:
        for line in f:
            if line.strip() == "" or line.startswith("#"):
                continue
            parts = line.strip().split()
            if len(parts) != 8:
                continue
            t = float(parts[0]) - 1680180030.90 + time_offset  # match eval_node logic
            x, y, z = map(float, parts[1:4])
            qx, qy, qz, qw = map(float, parts[4:8])
            gt_poses.append((t, np.array([x, y, z]), np.array([qx, qy, qz, qw])))
    # Find the first pose at or after t=0
    origin = None
    for t, pos, quat in gt_poses:
        if t >= 0:
            origin = pos
            origin_quat = quat
            break
    if origin is not None:
        # Offset all poses by the origin position and orientation
        origin_rot = R.from_quat(origin_quat)
        origin_rot_inv = origin_rot.inv()
        gt_poses = [
            (
                t,
                origin_rot_inv.apply(pos - origin),
                (R.from_quat(quat) * origin_rot_inv).as_quat()
            )
            for (t, pos, quat) in gt_poses
        ]

    # Remove all poses that happen before t=0
    gt_poses = [(t, pos, quat) for (t, pos, quat) in gt_poses if t >= 0]

    
    return gt_poses

class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.declare_parameter('ground_truth_path', '')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('gt_time_offset', 0.0)
        gt_path = self.get_parameter('ground_truth_path').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        gt_time_offset = self.get_parameter('gt_time_offset').get_parameter_value().double_value
        if not gt_path:
            self.get_logger().error("No ground_truth_path parameter provided!")
            raise RuntimeError("ground_truth_path parameter is required")
        self.gt_poses = load_ground_truth_tum(gt_path, time_offset=gt_time_offset)
        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth_pose', 10)
        self.path_pub = self.create_publisher(Path, '/ground_truth_path', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        self.idx = 0
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.slam_pose_received = False
        self.slam_pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.slam_pose_callback,
            1
        )
        self.get_logger().info(f"Publishing ground truth from {gt_path} at {self.publish_rate} Hz with time offset {gt_time_offset}")

    def slam_pose_callback(self, msg):
        self.slam_pose_received = True

    def timer_callback(self):
        if not self.slam_pose_received:
            return  # Wait until the first SLAM pose is received
        if self.idx >= len(self.gt_poses):
            self.get_logger().info("Finished publishing all ground truth poses.")
            return
        t, pos, quat = self.gt_poses[self.idx]
        now = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = "map"
        pose_msg.pose = Pose(
            position=Point(x=pos[0], y=pos[1], z=pos[2]),
            orientation=Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        )
        self.pose_pub.publish(pose_msg)
        self.path_msg.header.stamp = now
        self.path_msg.poses.append(pose_msg)
        self.path_pub.publish(self.path_msg)
        self.idx += 1

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()