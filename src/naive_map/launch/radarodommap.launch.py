import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='naive_map',
            executable='naive_mapper_node',
            name='naive_mapper',
            output='screen',
            parameters=[
                {'voxel_size': 0.1},
                {'map_frame': 'map'},
                {'robot_frame': 'base_link'},
                {'publish_rate': 1.0}
            ],
            remappings=[
                ('/Ego-Velocity', '/Ego-Velocity'),
                ('/Filtered_cloud', '/Filtered_cloud')
            ]
        )
    ])