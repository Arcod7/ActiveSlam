"""
Frontier-based exploration.

Prerequisites (must already be running):
  ros2 launch stonefish_groundtruth_mapping step3_octomap.launch.py

This adds:
  - frontier_extractor : /projected_map → /frontier_slam/goal + /frontier_slam/path
  - waypoint_controller: /frontier_slam/path + odometry → thruster setpoints

Optional arguments:
  depth       Target depth in NED metres (Z-down, so positive = below surface).
              If omitted, the controller locks the robot's depth on first odometry.
  odom_topic  Odometry topic for both nodes.
              Default: /StoneFish/Odometry (ground truth)
              Noisy:   /StoneFish/Odometry/noisy (requires odom_to_tf_noisy running)

Examples:
  ros2 launch frontier_slam frontier_slam.launch.py
  ros2 launch frontier_slam frontier_slam.launch.py depth:=8.0
  ros2 launch frontier_slam frontier_slam.launch.py odom_topic:=/StoneFish/Odometry/noisy

Visualise in RViz2:
  - MarkerArray  /frontier_slam/frontiers  (cyan = candidates, red = active goal)
  - Image        /frontier_slam/debug_image
  - OccupancyGrid /frontier_slam/inflated_map
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    depth = LaunchConfiguration('depth')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        depth_arg,
        odom_topic_arg,
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
    ])
