import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='naive_map',
            executable='simple_odom_node',  # Changed from naive_mapper_node
            name='simple_odom',             # Updated name to match
            output='screen',
            parameters=[
                {'map_frame': 'map'},
                {'robot_frame': 'base_link'}
            ],
            #remappings=[
            #    ('/ego_velocity', '/ego_vel_twist')
            #]
        )
    ])