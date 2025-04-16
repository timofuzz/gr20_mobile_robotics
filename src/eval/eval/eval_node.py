#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os
from ament_index_python.packages import get_package_share_directory
from .gt_publisher_node import load_ground_truth_tum


def find_closest_gt(gt_poses, t):
    """
    Finds the ground truth pose closest in time to the given timestamp t.
    """
    idx = np.argmin([abs(gt_t - t) for gt_t, _, _ in gt_poses])
    return gt_poses[idx]

class SlamEvalNode(Node):
    """
    Node that subscribes to a SLAM pose topic, compares it to ground truth,
    and accumulates error statistics for later plotting.
    """
    def __init__(self):
        super().__init__('slam_eval_node')
        # Declare and get the ground truth path parameter
        self.declare_parameter('ground_truth_path', '')
        self.declare_parameter('gt_time_offset', 0.0)  # Default offset in seconds
        gt_path = self.get_parameter('ground_truth_path').get_parameter_value().string_value
        gt_time_offset = self.get_parameter('gt_time_offset').get_parameter_value().double_value
        if not gt_path:
            self.get_logger().error("No ground_truth_path parameter provided!")
            raise RuntimeError("ground_truth_path parameter is required")
        # Pass the time offset to the loader
        self.gt_poses = load_ground_truth_tum(gt_path, time_offset=gt_time_offset)
        self.gt_time_offset = gt_time_offset
        self.errors = []  # List of (timestamp, position error, orientation error)
        self.times = []   # List of timestamps
        # Subscribe to the SLAM pose topic
        self.slam_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.slam_pose_callback,
            10
        )
        self.gt_pose_pub = self.create_publisher(PoseStamped, '/eval/ground_truth_pose', 10)
        self.get_logger().info(f"SLAM Evaluation Node started with ground truth: {gt_path}, time offset: {gt_time_offset}s")
    
    def slam_pose_callback(self, msg):
        # Compute the SLAM timestamp
        # Only compute slam_time the first time this callback runs
        if not hasattr(self, 'slam_time_initialized'):
            self.slam_time_initialized = True
            self.first_slam_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        slam_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.first_slam_time
        t = slam_time

        # Find closest ground truth pose
        gt_t, gt_pos, gt_quat = find_closest_gt(self.gt_poses, t)
        if abs(gt_t - t) > 0.5:
            self.get_logger().warn(f"No close ground truth for SLAM t={t:.2f}, closest gt_t={gt_t:.2f}")
            return

        # Publish the ground truth pose
        gt_pose_msg = PoseStamped()
        gt_pose_msg.header.stamp = msg.header.stamp
        gt_pose_msg.header.frame_id = "map"
        gt_pose_msg.pose.position.x = gt_pos[0]
        gt_pose_msg.pose.position.y = gt_pos[1]
        gt_pose_msg.pose.position.z = gt_pos[2]
        gt_pose_msg.pose.orientation.x = gt_quat[0]
        gt_pose_msg.pose.orientation.y = gt_quat[1]
        gt_pose_msg.pose.orientation.z = gt_quat[2]
        gt_pose_msg.pose.orientation.w = gt_quat[3]
        self.gt_pose_pub.publish(gt_pose_msg)

        # Extract SLAM pose
        slam_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        slam_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        # Compute position error (Euclidean distance)
        pos_err = np.linalg.norm(slam_pos - gt_pos)
        # Compute orientation error (angle between quaternions)
        r_slam = R.from_quat(slam_quat)
        r_gt = R.from_quat(gt_quat)
        rot_err = r_slam.inv() * r_gt
        ang_err = rot_err.magnitude()
        # Store errors and time
        self.errors.append((t, pos_err, ang_err))
        self.times.append(t)
        # Log the errors
        #self.get_logger().info(f"SLAM t={t:.2f} gt_t={gt_t:.2f} slam_pos={slam_pos} gt_pos={gt_pos}")
        #self.get_logger().info(f"t={t:.2f} pos_err={pos_err:.3f}m ang_err={np.degrees(ang_err):.2f}deg")
    
    def plot_errors(self):
        """
        Plots the position and orientation error over time.
        Called when the node is shutting down.
        Saves the plots into the images folder in the src directory.
        """
        if not self.errors:
            print("No errors to plot.")
            return
        times = [e[0] - self.errors[0][0] for e in self.errors]  # Relative time
        pos_errs = [e[1] for e in self.errors]
        ang_errs = [np.degrees(e[2]) for e in self.errors]
        plt.figure()
        plt.subplot(2,1,1)
        plt.plot(times, pos_errs)
        plt.ylabel("Position Error [m]")
        plt.subplot(2,1,2)
        plt.plot(times, ang_errs)
        plt.ylabel("Orientation Error [deg]")
        plt.xlabel("Time [s]")
        plt.tight_layout()

        # Save the plot to the images directory inside the eval package share directory
        package_share_dir = get_package_share_directory('eval')
        save_dir = os.path.join(package_share_dir, 'images')
        os.makedirs(save_dir, exist_ok=True)
        plot_path = os.path.join(save_dir, 'slam_eval_errors.png')
        plt.savefig(plot_path)
        print(f"Saved error plot to {plot_path}")
        plt.close()

def main(args=None):
    """
    Main entry point for the node.
    Spins the node and plots errors on shutdown.
    """
    rclpy.init(args=args)
    node = SlamEvalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_errors()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()