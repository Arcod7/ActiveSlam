from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    noise_profile_arg = DeclareLaunchArgument(
        'noise_profile', default_value='realistic',
        description='Noise profile to use (ideal, realistic, degraded)'
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
        parameters=[{'noise_profile_path': noise_file}]
    )
    
    imu_node = Node(
        package='slam_backend',
        executable='imu_sim',
        name='imu_sim',
        parameters=[{'noise_profile_path': noise_file}]
    )
    
    dvl_node = Node(
        package='slam_backend',
        executable='dvl_sim',
        name='dvl_sim',
        parameters=[{'noise_profile_path': noise_file}]
    )
    
    return LaunchDescription([
        noise_profile_arg,
        pressure_node,
        imu_node,
        dvl_node
    ])
