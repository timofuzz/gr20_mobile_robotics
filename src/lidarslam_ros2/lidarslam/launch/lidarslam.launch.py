import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    main_param_dir = launch.substitutions.LaunchConfiguration(
        'main_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'param',
            'lidarslam.yaml'))
    
    rviz_param_dir = launch.substitutions.LaunchConfiguration(
        'rviz_param_dir',
        default=os.path.join(
            get_package_share_directory('lidarslam'),
            'rviz',
            'mapping.rviz'))

    # Declare point cloud topic as a launch argument
    pointcloud_topic = launch.substitutions.LaunchConfiguration(
        'pointcloud_topic',
        default='/ouster/points'
    )

    mapping = launch_ros.actions.Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[main_param_dir],
        remappings=[('/input_cloud', pointcloud_topic)],
        output='screen'
    )

    tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','1','base_link','velodyne']
    )

    graphbasedslam = launch_ros.actions.Node(
        package='graph_based_slam',
        executable='graph_based_slam_node',
        parameters=[main_param_dir],
        output='screen'
    )
    
    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_param_dir]
    )

    # Print the point cloud topic being used
    log_pointcloud_topic = launch.actions.LogInfo(
        msg=['Using point cloud topic: ', pointcloud_topic]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'main_param_dir',
            default_value=main_param_dir,
            description='Full path to main parameter file to load'),
        launch.actions.DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/ouster/points',
            description='PointCloud2 topic to use for SLAM'
        ),
        log_pointcloud_topic,
        mapping,
        tf,
        graphbasedslam,
        rviz,
    ])