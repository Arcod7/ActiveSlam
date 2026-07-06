"""Full SLAM backend: simulated sensors (pressure/IMU/DVL) + dead-reckoning
fusion + GTSAM pose-graph correction.

Usage:
  ros2 launch slam_backend slam.launch.py noise_profile:=realistic

Does NOT start Stonefish, the point cloud pipeline, or the mapper — this is
included by bringup/demo.launch.py alongside those, with the ground-truth TF
broadcaster (odom_tf_sync) swapped out so pose_graph.py is the only node
broadcasting world_ned -> bluerov2/base_link.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    noise_profile_arg = DeclareLaunchArgument(
        'noise_profile', default_value='realistic',
        description='Noise profile to use (ideal, realistic, degraded)'
    )
    loop_closure_arg = DeclareLaunchArgument(
        'loop_closure', default_value='true',
        description='Enable loop-closure detection (false = odometry+scan-matching only, for A/B benchmarking)',
    )
    noise_seed_arg = DeclareLaunchArgument(
        'noise_seed', default_value='-1',
        description='Override the noise profile seed (-1 = use the profile default)',
    )

    pkg_share = FindPackageShare('slam_backend')
    noise_file = PathJoinSubstitution([
        pkg_share, 'config',
        ['noise_', LaunchConfiguration('noise_profile'), '.yaml']
    ])

    slam_backend_share = get_package_share_directory('slam_backend')
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_backend_share, 'launch', 'sensors_only.launch.py')
        ),
        launch_arguments={
            'noise_profile': LaunchConfiguration('noise_profile'),
            'noise_seed': LaunchConfiguration('noise_seed'),
        }.items(),
    )

    pose_graph_node = Node(
        package='slam_backend',
        executable='pose_graph',
        name='pose_graph',
        output='screen',
        parameters=[{
            'noise_profile_path': noise_file,
            'loop_closure_enabled': LaunchConfiguration('loop_closure'),
        }],
    )

    return LaunchDescription([
        noise_profile_arg, loop_closure_arg, noise_seed_arg,
        sensors,
        pose_graph_node,
    ])
