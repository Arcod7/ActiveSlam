"""Launch the safety gate and ArduSub adapter without autonomy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Build the hardware-acceptance launch description."""
    package_share = get_package_share_directory('frontier_slam')
    default_params = os.path.join(package_share, 'config', 'ardusub.yaml')
    odom_source = LaunchConfiguration('odom_source')
    from_mavlink = IfCondition(
        PythonExpression(["'", odom_source, "' == 'mavlink'"]))

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
            'odom_source',
            default_value='mavlink',
            choices=['mavlink', 'external'],
            description=(
                'mavlink starts mavlink_odometry, which turns the autopilot '
                'navigation estimate into odometry and TF. external means '
                'another node already publishes odom_topic.'),
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/mavlink/odometry',
            description='Real, continuously updated vehicle odometry topic.',
        ),
        DeclareLaunchArgument(
            'odom_connection_url',
            default_value='udpin:0.0.0.0:14561',
            description=(
                'Second BlueOS endpoint for mavlink_odometry. pymavlink binds '
                'the port, so this must differ from connection_url.'),
        ),
        DeclareLaunchArgument(
            'require_position',
            default_value='true',
            description=(
                'Whether ArduSub has a horizontal position solution (DVL or '
                'GPS in its EKF). false publishes depth and attitude only and '
                'marks X/Y unestimated, instead of mapping against an origin '
                'the EKF is not tracking.'),
        ),
        DeclareLaunchArgument(
            'require_odom',
            default_value='true',
            description=(
                'Whether the gate demands fresh odometry before passing any '
                'command. Only set false for teleop acceptance with no pose '
                'source at all — never for autonomy.'),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='ArduSub authority limits and timeout parameters.',
        ),
        Node(
            package='frontier_slam',
            executable='mavlink_odometry',
            name='mavlink_odometry',
            output='screen',
            condition=from_mavlink,
            parameters=[{
                'connection_url': LaunchConfiguration('odom_connection_url'),
                'odometry_topic': LaunchConfiguration('odom_topic'),
                'require_position': LaunchConfiguration('require_position'),
            }],
        ),
        Node(
            package='frontier_slam',
            executable='motion_safety_gate',
            name='motion_safety_gate',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                # No simulator pose to draw beside the estimate on hardware.
                'truth_odom_topic': LaunchConfiguration('odom_topic'),
                'require_odom': LaunchConfiguration('require_odom'),
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
