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
from launch_ros.parameter_descriptions import ParameterValue
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
    odom_coherent_noise_arg = DeclareLaunchArgument(
        'odom_coherent_noise', default_value='false',
        description='Size DVL scale/bias edge sigmas from run-long cumulative distance/time '
                    'instead of per-edge (see odom_noise.py). Not comparable to a run recorded '
                    'with this off -- it raises d-opt/sigma_xy for the same noise profile.',
    )
    initial_x_arg = DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='Initial dead-reckoning X position in world_ned (m)',
    )
    initial_y_arg = DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='Initial dead-reckoning Y position in world_ned (m)',
    )

    # Pose-graph structure and factor noise. Defaults mirror pose_graph.py's
    # own declare_parameter values, so passing none of these changes nothing.
    graph_args = [
        ('keyframe_dist_m', '0.5', float,
         'Distance travelled (m) before a new keyframe is added'),
        ('keyframe_angle_rad', '0.2', float,
         'Rotation (rad) before a new keyframe is added'),
        ('keyframe_max_per_cell', '3', int,
         'Keyframes kept per spatial cell — the cap a parked vehicle saturates'),
        ('loop_closure_radius_m', '5.0', float,
         'Radius (m) searched for loop-closure candidates'),
        ('loop_closure_min_gap', '20', int,
         'Minimum keyframe index gap before two poses may close a loop'),
        ('min_inlier_ratio', '0.3', float,
         'Registration inlier ratio a candidate closure must reach to be accepted'),
        ('scan_sigma_trans', '0.12', float,
         'Translational sigma (m) on a scan-matching factor'),
        ('scan_sigma_rot', '0.08', float,
         'Rotational sigma (rad) on a scan-matching factor'),
        ('odom_sigma_trans', '0.002', float,
         'Translational sigma (m) on a dead-reckoning odometry factor'),
        ('odom_sigma_rot', '0.02', float,
         'Rotational sigma (rad) on a dead-reckoning odometry factor'),
    ]
    graph_arg_actions = [
        DeclareLaunchArgument(name, default_value=default, description=doc)
        for name, default, _, doc in graph_args
    ]

    pkg_share = FindPackageShare('slam_backend')
    per_sensor_args = [
        DeclareLaunchArgument(
            f'noise_profile_{name}',
            default_value=LaunchConfiguration('noise_profile'),
            description=f'Noise profile for {name} only (default: noise_profile)')
        for name in ('pressure', 'imu', 'compass', 'dvl')
    ]

    def profile_file(arg_name):
        return PathJoinSubstitution([
            pkg_share, 'config',
            ['noise_', LaunchConfiguration(arg_name), '.yaml']
        ])

    noise_file = profile_file('noise_profile')

    sensors_arg = DeclareLaunchArgument(
        'sensors', default_value='sim', choices=['sim', 'external'],
        description=(
            'Who publishes /slam/sensors/*. sim runs the simulated '
            'pressure/IMU/compass/DVL nodes off the Stonefish pose; external '
            'means something else already fills those topics — on hardware, '
            'mavlink_odometry from the autopilot. Two publishers on one '
            'sensor topic is the same fault as two command publishers.'),
    )

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
            'sensors': LaunchConfiguration('sensors'),
        }.items(),
    )

    pose_graph_node = Node(
        package='slam_backend',
        executable='pose_graph',
        name='pose_graph',
        output='screen',
        parameters=[{
            'noise_profile_path': noise_file,
            # Same per-sensor mix the sims run with — the graph's noise models
            # must describe the noise actually injected.
            **{f'noise_profile_path_{n}': profile_file(f'noise_profile_{n}')
               for n in ('pressure', 'imu', 'compass', 'dvl')},
            'loop_closure_enabled': LaunchConfiguration('loop_closure'),
            'map_rebuild_enabled': LaunchConfiguration('map_rebuild'),
            'odom_coherent_noise': LaunchConfiguration('odom_coherent_noise'),
            **{name: ParameterValue(LaunchConfiguration(name), value_type=kind)
               for name, _, kind, _ in graph_args},
        }],
    )

    return LaunchDescription([
        noise_profile_arg, loop_closure_arg, noise_seed_arg, map_rebuild_arg,
        odom_coherent_noise_arg,
        *per_sensor_args,
        *graph_arg_actions,
        initial_x_arg, initial_y_arg, sensors_arg,
        sensors,
        pose_graph_node,
    ])
