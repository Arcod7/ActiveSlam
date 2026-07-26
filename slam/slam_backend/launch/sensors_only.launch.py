from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    noise_profile_arg = DeclareLaunchArgument(
        'noise_profile', default_value='realistic',
        description='Noise profile to use (ideal, sonar_only, odom_pos_only, odom_only, '
                    'realistic, realistic_no_reverb, degraded)'
    )
    noise_seed_arg = DeclareLaunchArgument(
        'noise_seed', default_value='-1',
        description='Override the noise profile seed (-1 = use the profile default)',
    )
    initial_x_arg = DeclareLaunchArgument(
        'initial_x', default_value='0.0',
        description='Initial dead-reckoning X position in world_ned (m)',
    )
    initial_y_arg = DeclareLaunchArgument(
        'initial_y', default_value='0.0',
        description='Initial dead-reckoning Y position in world_ned (m)',
    )

    # Per-sensor overrides. Each sim node reads only its own section of the
    # YAML, so pointing them at different profiles isolates one error source
    # without any file merging. Default is the master profile.
    per_sensor_args = [
        DeclareLaunchArgument(
            f'noise_profile_{name}',
            default_value=LaunchConfiguration('noise_profile'),
            description=f'Noise profile for {name} only (default: noise_profile)')
        for name in ('pressure', 'imu', 'compass', 'dvl')
    ]

    pkg_share = FindPackageShare('slam_backend')

    def profile_file(arg_name):
        return PathJoinSubstitution([
            pkg_share, 'config',
            ['noise_', LaunchConfiguration(arg_name), '.yaml']
        ])

    noise_file = profile_file('noise_profile')

    pressure_node = Node(
        package='slam_backend',
        executable='pressure_sim',
        name='pressure_sim',
        parameters=[{
            'noise_profile_path': profile_file('noise_profile_pressure'),
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    imu_node = Node(
        package='slam_backend',
        executable='imu_sim',
        name='imu_sim',
        parameters=[{
            'noise_profile_path': profile_file('noise_profile_imu'),
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    compass_node = Node(
        package='slam_backend',
        executable='compass_sim',
        name='compass_sim',
        parameters=[{
            'noise_profile_path': profile_file('noise_profile_compass'),
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    dvl_node = Node(
        package='slam_backend',
        executable='dvl_sim',
        name='dvl_sim',
        parameters=[{
            'noise_profile_path': profile_file('noise_profile_dvl'),
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    dead_reckoning_node = Node(
        package='slam_backend',
        executable='dead_reckoning',
        name='dead_reckoning',
        parameters=[{
            'noise_profile_path': noise_file,
            'initial_x': LaunchConfiguration('initial_x'),
            'initial_y': LaunchConfiguration('initial_y'),
        }],
    )

    return LaunchDescription([
        noise_profile_arg, noise_seed_arg, initial_x_arg, initial_y_arg,
        *per_sensor_args,
        pressure_node,
        imu_node, compass_node,
        dvl_node,
        dead_reckoning_node,
    ])
