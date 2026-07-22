"""Launch the safety gate and ArduSub adapter without autonomy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the hardware-acceptance launch description."""
    package_share = get_package_share_directory('frontier_slam')
    default_params = os.path.join(package_share, 'config', 'ardusub.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'backend',
            default_value='manual_control',
            choices=['manual_control', 'local_ned_velocity'],
            description=(
                'MAVLink command interface. Start real testing with '
                'manual_control; local_ned_velocity requires a healthy '
                'ArduSub Guided/EKF position solution.'),
        ),
        DeclareLaunchArgument(
            'connection_url',
            default_value='udpin:0.0.0.0:14560',
            description='Dedicated BlueOS/pymavlink connection URL.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            description='Real, continuously updated vehicle odometry topic.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ArduSub authority limits and timeout parameters.',
        ),
        Node(
            package='frontier_slam',
            executable='motion_safety_gate',
            name='motion_safety_gate',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'start_enabled': False,
            }],
        ),
        Node(
            package='frontier_slam',
            executable='ardusub_adapter',
            name='ardusub_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'backend': LaunchConfiguration('backend'),
                    'connection_url': LaunchConfiguration('connection_url'),
                },
            ],
        ),
    ])
