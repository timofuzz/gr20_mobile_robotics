#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import open3d as o3d
import threading
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class NaiveMapper(Node):
    def __init__(self):
        super().__init__('naive_mapper')
        
        # Parameters
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('publish_rate', 1.0)  # map publishing rate in Hz
        
        self.voxel_size = self.get_parameter('voxel_size').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Create QoS profiles
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialize subscribers
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/Ego-Velocity',
            self.velocity_callback,
            qos
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/Filtered_cloud',
            self.pointcloud_callback,
            qos
        )
        
        # Initialize publishers
        self.map_publisher = self.create_publisher(
            MarkerArray, 
            '/voxel_map', 
            10
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize Open3D voxel grid
        self.voxel_grid = o3d.geometry.VoxelGrid()
        self.point_cloud_map = o3d.geometry.PointCloud()
        
        # Pose tracking
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.current_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion x, y, z, w
        self.last_velocity_time = None
        
        # Mutex for thread safety
        self.lock = threading.Lock()
        
        # Start map publishing thread
        self.stop_thread = False
        self.map_thread = threading.Thread(target=self.map_publishing_thread)
        self.map_thread.start()
        
        self.get_logger().info('Naive Mapper node initialized')
        
    def velocity_callback(self, msg):
        """Process incoming velocity message and update pose"""
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.last_velocity_time is not None:
            dt = current_time - self.last_velocity_time
            
            # Linear velocity in x, y, z
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            vz = msg.twist.linear.z
            
            # Angular velocity in x, y, z
            wx = msg.twist.angular.x
            wy = msg.twist.angular.y
            wz = msg.twist.angular.z
            
            # Simple integration for position
            with self.lock:
                # Update position
                self.current_position[0] += vx * dt
                self.current_position[1] += vy * dt
                self.current_position[2] += vz * dt
                
                # Update orientation using quaternion integration (scipy)
                axis = np.array([wx, wy, wz])
                angle = np.linalg.norm(axis) * dt
                if angle > 1e-10:
                    axis = axis / np.linalg.norm(axis)
                    q_delta = R.from_rotvec(axis * angle)
                    q_current = R.from_quat(self.current_orientation)
                    q_new = q_delta * q_current
                    self.current_orientation = q_new.as_quat()  # [x, y, z, w]
                
            # Broadcast transform
            self.broadcast_transform()
            
            # Publish odometry
            self.publish_odometry(msg.header.stamp)
        
        self.last_velocity_time = current_time
        
    def pointcloud_callback(self, msg):
        """Process incoming point cloud and add to map"""
        try:
            # Convert ROS PointCloud2 to numpy array
            pc_data = pc2.read_points_list(msg, field_names=("x", "y", "z"))
            points = np.array([[p.x, p.y, p.z] for p in pc_data])
            
            if len(points) == 0:
                return
                
            # Create Open3D point cloud
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            
            # Transform point cloud to map frame using current pose
            with self.lock:
                # Create transformation matrix from current orientation and position
                rot = R.from_quat(self.current_orientation)
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rot.as_matrix()
                transform_matrix[:3, 3] = self.current_position
                
                # Apply transformation
                cloud.transform(transform_matrix)
            
            # Voxelize the point cloud to reduce memory usage
            voxel_down_cloud = cloud.voxel_down_sample(voxel_size=self.voxel_size)
            
            # Add to global map
            with self.lock:
                if len(self.point_cloud_map.points) == 0:
                    self.point_cloud_map = voxel_down_cloud
                else:
                    self.point_cloud_map += voxel_down_cloud
                    # Down-sample the combined cloud
                    self.point_cloud_map = self.point_cloud_map.voxel_down_sample(voxel_size=self.voxel_size)
                
                # Create voxel grid from point cloud
                self.voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(
                    self.point_cloud_map, voxel_size=self.voxel_size)
                
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def broadcast_transform(self):
        """Broadcast the transform from map to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.robot_frame
        
        t.transform.translation.x = self.current_position[0]
        t.transform.translation.y = self.current_position[1]
        t.transform.translation.z = self.current_position[2]
        
        # Note: geometry_msgs uses [x, y, z, w]
        t.transform.rotation.x = self.current_orientation[0]
        t.transform.rotation.y = self.current_orientation[1]
        t.transform.rotation.z = self.current_orientation[2]
        t.transform.rotation.w = self.current_orientation[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odometry(self, stamp):
        """Publish odometry message"""
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.robot_frame
        
        odom.pose.pose.position.x = self.current_position[0]
        odom.pose.pose.position.y = self.current_position[1]
        odom.pose.pose.position.z = self.current_position[2]
        
        odom.pose.pose.orientation.x = self.current_orientation[0]
        odom.pose.pose.orientation.y = self.current_orientation[1]
        odom.pose.pose.orientation.z = self.current_orientation[2]
        odom.pose.pose.orientation.w = self.current_orientation[3]
        
        self.odom_publisher.publish(odom)
    
    def map_publishing_thread(self):
        """Thread to periodically publish the map visualization"""
        rate = self.create_rate(self.publish_rate)
        
        while rclpy.ok() and not self.stop_thread:
            with self.lock:
                if self.voxel_grid is not None and hasattr(self.voxel_grid, 'get_voxels'):
                    try:
                        # Create marker array for visualization
                        marker_array = MarkerArray()
                        voxels = self.voxel_grid.get_voxels()
                        
                        # Clear previous markers
                        clear_marker = Marker()
                        clear_marker.action = Marker.DELETEALL
                        marker_array.markers.append(clear_marker)
                        
                        for i, voxel in enumerate(voxels):
                            marker = Marker()
                            marker.header.frame_id = self.map_frame
                            marker.header.stamp = self.get_clock().now().to_msg()
                            marker.ns = "voxel_map"
                            marker.id = i
                            marker.type = Marker.CUBE
                            marker.action = Marker.ADD
                            
                            # Voxel center
                            voxel_center = voxel.grid_index * self.voxel_size
                            marker.pose.position.x = voxel_center[0]
                            marker.pose.position.y = voxel_center[1]
                            marker.pose.position.z = voxel_center[2]
                            
                            # Orientation - identity
                            marker.pose.orientation.w = 1.0
                            
                            # Scale - voxel size
                            marker.scale.x = self.voxel_size
                            marker.scale.y = self.voxel_size
                            marker.scale.z = self.voxel_size
                            
                            # Color
                            marker.color.r = 0.0
                            marker.color.g = 1.0
                            marker.color.b = 0.0
                            marker.color.a = 0.8  # Semi-transparent
                            
                            # Lifetime
                            marker.lifetime.sec = 1
                            
                            marker_array.markers.append(marker)
                        
                        # Publish the marker array
                        self.map_publisher.publish(marker_array)
                        
                    except Exception as e:
                        self.get_logger().error(f'Error publishing map: {str(e)}')
            
            rate.sleep()
    
    def destroy_node(self):
        self.stop_thread = True
        if self.map_thread.is_alive():
            self.map_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    naive_mapper = NaiveMapper()
    
    try:
        rclpy.spin(naive_mapper)
    except KeyboardInterrupt:
        pass
    finally:
        naive_mapper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()