import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare which mode to run
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='lidar',
        description="Choose 'lidar', 'radar_naive', or 'radar_slam'"
    )
    gt_arg = DeclareLaunchArgument(
        'ground_truth_path',
        default_value='',
        description='Path to ground truth TUM file'
    )

    # Paths to your launch files
    pkg_lidarslam = os.path.join(
        get_package_share_directory('lidarslam'), 'launch', 'lidarslam.launch.py')
    pkg_radarodom = os.path.join(
        get_package_share_directory('naive_map'), 'launch', 'radarodommap.launch.py')

    def launch_setup(context, *args, **kwargs):
        mode = LaunchConfiguration('mode').perform(context)
        ground_truth_path = LaunchConfiguration('ground_truth_path').perform(context)
        actions = []

        # Launch the selected SLAM/odometry node
        if mode == 'lidar':
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pkg_lidarslam)
                )
            )
        elif mode == 'radar_naive':
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pkg_radarodom)
                )
            )
        elif mode == 'radar_slam':
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(pkg_lidarslam),
                    launch_arguments={'registration_method': 'RADAR'}.items()
                )
            )
        else:
            raise RuntimeError(f"Unknown mode: {mode}")

        # Always launch the evaluation node
        actions.append(
            Node(
                package='eval',
                executable='eval_node',
                name='slam_eval_node',
                output='screen',
                parameters=[
                    {'ground_truth_path': ground_truth_path},
                    {'gt_time_offset': 120.0 - 155.71 - 0}  # or whatever offset you need
                ],
                remappings=[
                    #('/current_pose', '/your_topic')  # Change '/your_topic' to your actual topic
                ]
            )
        )

        # Add the ground truth publisher node
        # actions.append(
        #     Node(
        #         package='eval',
        #         executable='gt_publisher_node',
        #         name='ground_truth_publisher',
        #         output='screen',
        #         parameters=[
        #             {'ground_truth_path': ground_truth_path},
        #             {'publish_rate': 10.0},
        #             {'gt_time_offset': 120.0 - 155.71}  # or whatever offset you need
        #         ]
        #     )
        # )

        return actions

    return LaunchDescription([
        mode_arg,
        gt_arg,
        OpaqueFunction(function=launch_setup)
    ])