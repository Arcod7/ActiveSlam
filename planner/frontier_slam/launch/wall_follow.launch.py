"""
Wall-normal following motion mode.

Prerequisites (must already be running — the wall follower consumes TSDF
surface normals, so the OctoMap stack is not enough):
  ros2 launch stonefish_groundtruth_mapping tsdf.launch.py

This adds:
  - wall_follower: /tsdf/surface_normals_cloud + odometry → thruster setpoints.
    Faces the nearest mapped wall along its TSDF normal, holds `standoff`
    metres off it, and strafes sideways along the wall.

The follower only knows walls the TSDF has already reconstructed.  From a
cold start it scans in place until the mapper produces surface within range
(~8 m); spawn the robot near structure, or teleop it close first.

Optional arguments:
  standoff       Perpendicular wall distance to hold, metres (default 1.5).
  tangent_speed  Strafe speed along the wall, m/s-ish setpoint (default 0.15).
  direction      +1 = strafe to the robot's right while facing the wall,
                 -1 = left (default 1).
  depth          Target depth in NED metres. Omit to lock from first odometry.
  odom_topic     Default /StoneFish/Odometry (ground truth).

Examples:
  ros2 launch frontier_slam wall_follow.launch.py
  ros2 launch frontier_slam wall_follow.launch.py standoff:=2.0 direction:=-1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    standoff_arg = DeclareLaunchArgument(
        'standoff', default_value='1.5',
        description='Perpendicular distance to hold off the wall, metres.',
    )
    tangent_speed_arg = DeclareLaunchArgument(
        'tangent_speed', default_value='0.15',
        description='Strafe speed along the wall.',
    )
    direction_arg = DeclareLaunchArgument(
        'direction', default_value='1',
        description="+1 = strafe right while facing the wall, -1 = left.",
    )
    depth_arg = DeclareLaunchArgument(
        'depth', default_value='-1.0',
        description='Target depth in NED metres; -1 locks from first odometry.',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/StoneFish/Odometry',
        description='Odometry topic. Use /StoneFish/Odometry/noisy for noisy mode.',
    )

    return LaunchDescription([
        standoff_arg, tangent_speed_arg, direction_arg, depth_arg, odom_topic_arg,
        Node(
            package='frontier_slam',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            parameters=[{
                'standoff_m':     LaunchConfiguration('standoff'),
                'tangent_speed':  LaunchConfiguration('tangent_speed'),
                'direction':      LaunchConfiguration('direction'),
                'depth_setpoint': LaunchConfiguration('depth'),
                'odom_topic':     LaunchConfiguration('odom_topic'),
            }],
        ),
    ])
