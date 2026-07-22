"""Standalone benchmark + map-metrics launch — attach to an already-running
SLAM session (bringup demo.launch.py slam:=slam) to log ATE/RPE/map-quality
and write TUM trajectory files, without restarting the sim/SLAM stack.

Usage:
  ros2 launch eval_tools eval.launch.py
  ros2 launch eval_tools eval.launch.py output_dir:=/path/to/run_dir mapper:=tsdf

output_dir is resolved ONCE here (an OpaqueFunction, evaluated at launch-file
parse time) rather than each node independently defaulting to its own
timestamp -- otherwise benchmark.py and map_metrics.py could each pick a
different `eval/runs/<timestamp>/` a few hundred ms apart and never see
each other's output.
"""
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from eval_tools.run_paths import new_run_dir


def _launch_eval_nodes(context, *args, **kwargs):
    output_dir = LaunchConfiguration('output_dir').perform(context)
    if not output_dir:
        output_dir = new_run_dir(time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(output_dir, exist_ok=True)

    benchmark_node = Node(
        package='eval_tools',
        executable='benchmark',
        name='benchmark',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'rpe_delta': LaunchConfiguration('rpe_delta'),
        }],
    )

    map_metrics_node = Node(
        package='eval_tools',
        executable='map_metrics',
        name='map_metrics',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'mapper': LaunchConfiguration('mapper'),
        }],
    )

    return [benchmark_node, map_metrics_node]


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='',
        description='Directory to write TUM/CSV output to (default: timestamped dir under eval/runs)',
    )
    rpe_delta_arg = DeclareLaunchArgument(
        'rpe_delta', default_value='1',
        description='Number of matched pose pairs between RPE samples',
    )
    mapper_arg = DeclareLaunchArgument(
        'mapper', default_value='octomap', choices=['octomap', 'tsdf'],
        description='Map backend map_metrics.py should score against the ground-truth map',
    )

    return LaunchDescription([
        output_dir_arg,
        rpe_delta_arg,
        mapper_arg,
        OpaqueFunction(function=_launch_eval_nodes),
    ])
