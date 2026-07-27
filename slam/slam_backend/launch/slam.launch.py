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
        description='Noise profile to use (ideal, sonar_only, odom_pos_only, odom_only, '
                    'realistic, realistic_no_reverb, degraded, degraded_no_reverb)'
    )
    loop_closure_arg = DeclareLaunchArgument(
        'loop_closure', default_value='true',
        description='Enable loop-closure detection (false = odometry+scan-matching only, for A/B benchmarking)',
    )
    noise_seed_arg = DeclareLaunchArgument(
        'noise_seed', default_value='-1',
        description='Override the noise profile seed (-1 = use the profile default)',
    )
    map_rebuild_arg = DeclareLaunchArgument(
        'map_rebuild', default_value='false',
        description='Rebuild the belief TSDF map from corrected keyframe poses after a big loop closure',
    )
    initial_x_arg = DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='Initial dead-reckoning X position in world_ned (m)',
    )
    initial_y_arg = DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='Initial dead-reckoning Y position in world_ned (m)',
    )

    pkg_share = FindPackageShare('slam_backend')
    per_sensor_args = [
        DeclareLaunchArgument(
            f'noise_profile_{name}',
            default_value=LaunchConfiguration('noise_profile'),
            description=f'Noise profile for {name} only (default: noise_profile)')
        for name in ('pressure', 'imu', 'compass', 'dvl')
    ]

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
            **{f'noise_profile_{n}': LaunchConfiguration(f'noise_profile_{n}')
               for n in ('pressure', 'imu', 'compass', 'dvl')},
            'noise_seed': LaunchConfiguration('noise_seed'),
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
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
            'map_rebuild_enabled': LaunchConfiguration('map_rebuild'),
        }],
    )

    return LaunchDescription([
        noise_profile_arg, loop_closure_arg, noise_seed_arg, map_rebuild_arg,
        *per_sensor_args,
        initial_x_arg, initial_y_arg,
        sensors,
        pose_graph_node,
    ])
