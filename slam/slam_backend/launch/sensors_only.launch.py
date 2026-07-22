from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    noise_profile_arg = DeclareLaunchArgument(
        'noise_profile', default_value='realistic',
        description='Noise profile to use (ideal, sonar_only, odom_pos_only, odom_only, realistic, degraded)'
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

    pressure_node = Node(
        package='slam_backend',
        executable='pressure_sim',
        name='pressure_sim',
        parameters=[{
            'noise_profile_path': noise_file,
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    imu_node = Node(
        package='slam_backend',
        executable='imu_sim',
        name='imu_sim',
        parameters=[{
            'noise_profile_path': noise_file,
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    compass_node = Node(
        package='slam_backend',
        executable='compass_sim',
        name='compass_sim',
        parameters=[{
            'noise_profile_path': noise_file,
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    dvl_node = Node(
        package='slam_backend',
        executable='dvl_sim',
        name='dvl_sim',
        parameters=[{
            'noise_profile_path': noise_file,
            'noise_seed': LaunchConfiguration('noise_seed'),
        }]
    )

    dead_reckoning_node = Node(
        package='slam_backend',
        executable='dead_reckoning',
        name='dead_reckoning',
        parameters=[{
            'noise_profile_path': noise_file,
        }],
    )

    return LaunchDescription([
        noise_profile_arg, noise_seed_arg,
        pressure_node,
        imu_node, compass_node,
        dvl_node,
        dead_reckoning_node,
    ])
