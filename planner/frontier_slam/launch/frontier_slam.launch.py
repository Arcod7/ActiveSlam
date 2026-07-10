"""
Frontier-based exploration.

Prerequisites (must already be running):
  ros2 launch stonefish_groundtruth_mapping octomap.launch.py

This adds:
  - frontier_extractor : /projected_map → /frontier_slam/goal + /frontier_slam/path
  - waypoint_controller: /frontier_slam/path + odometry → thruster setpoints
  - revisit_planner (revisit:=true only): /slam/dopt → suspends frontier_extractor
    and drives a forced revisit to close a loop when pose uncertainty grows
    (needs a SLAM pose source — see revisit_planner.py)
  - drift_return_scenario (scenario:=drift_return only): a fixed, scripted
    leave-and-return waypoint sequence for watching loop closure fire on
    return (docs/plans/plan.md T1.1) — same suspend/goal interface as
    revisit_planner, see drift_return_scenario.py

Optional arguments:
  depth       Target depth in NED metres (Z-down, so positive = below surface).
              If omitted, the controller locks the robot's depth on first odometry.
  odom_topic  Odometry topic for both nodes.
              Default: /StoneFish/Odometry (ground truth)
              Noisy:   /StoneFish/Odometry/noisy (requires odom_to_tf_noisy running)
  revisit     Start revisit_planner. Default: false.
  scenario    Scripted evaluation scenario: none or drift_return. Default: none.
  scenario_out_dx/scenario_out_dy
              Outbound leg offset (m) from the start position for
              scenario:=drift_return. Default: 15.0 / 0.0.

Examples:
  ros2 launch frontier_slam frontier_slam.launch.py
  ros2 launch frontier_slam frontier_slam.launch.py depth:=8.0
  ros2 launch frontier_slam frontier_slam.launch.py odom_topic:=/StoneFish/Odometry/noisy
  ros2 launch frontier_slam frontier_slam.launch.py revisit:=true
  ros2 launch frontier_slam frontier_slam.launch.py scenario:=drift_return

Visualise in RViz2:
  - MarkerArray  /frontier_slam/frontiers  (cyan = candidates, red = active goal)
  - Image        /frontier_slam/debug_image
  - OccupancyGrid /frontier_slam/inflated_map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    depth_arg = DeclareLaunchArgument(
        'depth',
        default_value='-1.0',
        description=(
            'Target depth in NED metres (e.g. depth:=8.0). '
            'Omit (or pass depth:=-1) to lock depth automatically from the first odometry reading.'
        ),
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/StoneFish/Odometry',
        description='Odometry topic for pose. Use /StoneFish/Odometry/noisy for noisy mode.',
    )
    revisit_arg = DeclareLaunchArgument(
        'revisit',
        default_value='false',
        description='Start revisit_planner (needs a SLAM pose source, e.g. bringup slam:=slam).',
    )
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='none',
        choices=['none', 'drift_return'],
        description='Scripted evaluation scenario: none or drift_return (docs/plans/plan.md T1.1).',
    )
    scenario_out_dx_arg = DeclareLaunchArgument(
        'scenario_out_dx',
        default_value='15.0',
        description='drift_return: outbound leg X offset (m) from the captured start position.',
    )
    scenario_out_dy_arg = DeclareLaunchArgument(
        'scenario_out_dy',
        default_value='0.0',
        description='drift_return: outbound leg Y offset (m) from the captured start position.',
    )
    depth = LaunchConfiguration('depth')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        depth_arg,
        odom_topic_arg,
        revisit_arg,
        scenario_arg,
        scenario_out_dx_arg,
        scenario_out_dy_arg,
        Node(
            package='frontier_slam',
            executable='frontier_extractor',
            name='frontier_extractor',
            output='screen',
            parameters=[{'odom_topic': odom_topic}],
        ),
        Node(
            package='frontier_slam',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen',
            parameters=[{'depth_setpoint': depth, 'odom_topic': odom_topic}],
        ),
        Node(
            package='frontier_slam',
            executable='revisit_planner',
            name='revisit_planner',
            output='screen',
            parameters=[{'odom_topic': odom_topic}],
            condition=IfCondition(LaunchConfiguration('revisit')),
        ),
        Node(
            package='frontier_slam',
            executable='drift_return_scenario',
            name='drift_return_scenario',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'out_dx': LaunchConfiguration('scenario_out_dx'),
                'out_dy': LaunchConfiguration('scenario_out_dy'),
            }],
            condition=LaunchConfigurationEquals('scenario', 'drift_return'),
        ),
    ])
