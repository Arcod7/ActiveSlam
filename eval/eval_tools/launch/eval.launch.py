"""Standalone benchmark node launch — attach to an already-running SLAM
session (bringup demo.launch.py slam:=slam) to log ATE/RPE and write TUM
trajectory files, without restarting the sim/SLAM stack.

Usage:
  ros2 launch eval_tools eval.launch.py
  ros2 launch eval_tools eval.launch.py output_dir:=/path/to/run_dir
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='',
        description='Directory to write TUM/CSV output to (default: timestamped dir under eval/runs)',
    )
    rpe_delta_arg = DeclareLaunchArgument(
        'rpe_delta', default_value='1',
        description='Number of matched pose pairs between RPE samples',
    )

    benchmark_node = Node(
        package='eval_tools',
        executable='benchmark',
        name='benchmark',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'rpe_delta': LaunchConfiguration('rpe_delta'),
        }],
    )

    return LaunchDescription([
        output_dir_arg,
        rpe_delta_arg,
        benchmark_node,
    ])
