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
from launch_ros.parameter_descriptions import ParameterValue

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
            'rpe_delta': ParameterValue(
                LaunchConfiguration('rpe_delta'), value_type=float),
            'preset': LaunchConfiguration('preset'),
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
            # Mirrors the map's cell size: explored extent is counted on this
            # grid, so a run mapped at 0.15 m must not be binned at 0.2 m.
            'voxel_size': ParameterValue(
                LaunchConfiguration('voxel_size'), value_type=float),
        }],
    )

    # The gt map mirrors the belief backend (gt_map.launch.py), so a gt octomap
    # to snapshot exists only when mapper:=octomap. Under tsdf the belief map
    # no longer runs any octomap_server (the frontier planning map is now
    # derived from the TSDF grid), so blank the belief octomap service too —
    # otherwise map_saver blocks up to 30 s per save shelling out to
    # octomap_saver_node against a dead /octomap_binary.
    mapper = LaunchConfiguration('mapper').perform(context)
    is_octomap = mapper == 'octomap'
    gt_octomap_service = '/gt/octomap_binary' if is_octomap else ''
    belief_octomap_service = '/octomap_binary' if is_octomap else ''
    map_saver_node = Node(
        package='eval_tools',
        executable='map_saver',
        name='map_saver',
        output='screen',
        parameters=[{
            'output_dir': output_dir,
            'mapper': LaunchConfiguration('mapper'),
            'gt_octomap_service': gt_octomap_service,
            'belief_octomap_service': belief_octomap_service,
        }],
    )

    return [benchmark_node, map_metrics_node, map_saver_node]


def generate_launch_description():
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='',
        description='Directory to write TUM/CSV output to (default: timestamped dir under eval/runs)',
    )
    rpe_delta_arg = DeclareLaunchArgument(
        'rpe_delta', default_value='1.0',
        description='Fixed temporal separation between RPE samples, in seconds',
    )
    mapper_arg = DeclareLaunchArgument(
        'mapper', default_value='octomap', choices=['octomap', 'tsdf'],
        description='Map backend map_metrics.py should score against the ground-truth map',
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', default_value='0.2',
        description='Map cell size in metres; mirrors the belief map',
    )
    preset_arg = DeclareLaunchArgument(
        'preset', default_value='',
        description='Launcher preset name to show on the Eval HUD (e.g. loop_closure_revisit)',
    )

    return LaunchDescription([
        output_dir_arg,
        rpe_delta_arg,
        mapper_arg,
        voxel_size_arg,
        preset_arg,
        OpaqueFunction(function=_launch_eval_nodes),
    ])
