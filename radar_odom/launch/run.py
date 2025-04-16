import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('radar_odom')

    radar_pcl_config = os.path.join(package_dir, 'config', 'radar_pcl_config.yaml')
    graph_slam_config = os.path.join(package_dir, 'config', 'graph_slam_config.yaml')

    radar_pcl_processor = Node(
        package='radar_odom',
        executable='radar_pcl_processor',
        output='screen',
        name='radar_pcl_processor',
        parameters=[radar_pcl_config]
    )

    optimizer = Node(
        package='radar_odom',
        executable='optimizer',
        output='screen',
        name='optimizer',
        parameters=[graph_slam_config]
    )

    baselink_tf = Node(
        package='radar_odom',
        executable='baselink_tf',
        name='baselink_tf',
        parameters=[{'topic_name': '/odometry'}]
    )

    record = Node(
        package='radar_odom',
        executable='record',
        name='record'
    )

    nodes_to_execute = [radar_pcl_processor, optimizer, record, baselink_tf]

    return LaunchDescription(nodes_to_execute)

