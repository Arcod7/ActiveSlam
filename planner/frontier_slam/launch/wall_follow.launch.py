"""
Wall-guided motion executor.

This launch starts only the motion executor. A planner must already publish a
goal and path on the configured topics; normally use
`bringup demo.launch.py mode:=frontier motion:=walllooking mapper:=tsdf`
instead. The executor does not select goals and reports BLOCKED through its
status topic when it cannot follow the requested route along a mapped wall.

This adds:
  - wall_follower: planner path + TSDF surface normals + odometry → normalized
    body demand. Follows the planner path, using the nearest mapped wall as a
    soft travel and viewing preference while holding `standoff` metres off it.

The executor only knows walls the TSDF has already reconstructed. If no usable
wall supports an active planner route within its timeout, it holds position and
reports BLOCKED; the planner decides what to try next.

Optional arguments:
  standoff       Perpendicular wall distance to hold, metres (default 1.5).
  tangent_speed  Strafe speed along the wall, m/s-ish setpoint (default 0.15).
  direction      Tie-break direction if the planner waypoint is exactly
                 perpendicular to the wall (default +1).
  depth          Target depth in NED metres. Omit to lock from first odometry.
  odom_topic     Default /StoneFish/Odometry (ground truth).

  path_influence
                 Travel blend: 0 = wall tangent; 1 = planner path (default 0.70).
  path_look_offset_deg
                 Turn the path-derived look heading toward the wall (default 30).
  wall_normal_offset_deg
                 Turn the wall-derived look heading toward the path (default 0).
  path_heading_weight
                 Look-heading blend: 0 = wall-derived; 1 = path-derived
                 (default 0.35).

  goal_topic/path_topic/status_topic/command_topic/safe_command_topic/thruster_topic
                 Configurable planner/motion interface topics.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_parameter(name: str) -> ParameterValue:
    """Resolve a launch argument as a ROS double, including whole numbers."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _bool_parameter(name: str) -> ParameterValue:
    """Resolve a launch argument as a ROS boolean."""
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


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
    goal_topic_arg = DeclareLaunchArgument(
        'goal_topic', default_value='/frontier_slam/goal',
        description='Planner goal topic (geometry_msgs/PointStamped).',
    )
    path_topic_arg = DeclareLaunchArgument(
        'path_topic', default_value='/frontier_slam/path',
        description='Planner path topic (nav_msgs/Path).',
    )
    status_topic_arg = DeclareLaunchArgument(
        'status_topic', default_value='/motion/status',
        description='Motion status topic (std_msgs/String).',
    )
    thruster_topic_arg = DeclareLaunchArgument(
        'thruster_topic', default_value='/bluerov2/controller/thruster_setpoints_sim',
        description='Safety-gated actuator output topic.',
    )
    command_topic_arg = DeclareLaunchArgument(
        'command_topic', default_value='/motion/body_command',
        description='Normalized body demand consumed by the safety gate.',
    )
    safe_command_topic_arg = DeclareLaunchArgument(
        'safe_command_topic', default_value='/motion/body_command_safe',
        description='Gated body demand consumed by the simulation mixer.',
    )
    safety_start_enabled_arg = DeclareLaunchArgument(
        'safety_start_enabled', default_value='false',
        choices=['true', 'false'],
        description='Start the safety gate enabled (false is the safe default).',
    )
    path_influence_arg = DeclareLaunchArgument(
        'path_influence', default_value='0.70',
        description='Travel blend: 0=wall tangent, 1=planner path direction.',
    )
    path_look_offset_arg = DeclareLaunchArgument(
        'path_look_offset_deg', default_value='30.0',
        description='Degrees to turn from the planner bearing toward the selected wall.',
    )
    wall_normal_offset_arg = DeclareLaunchArgument(
        'wall_normal_offset_deg', default_value='0.0',
        description='Degrees to turn from the wall-facing normal toward the planner bearing.',
    )
    path_heading_weight_arg = DeclareLaunchArgument(
        'path_heading_weight', default_value='0.35',
        description='Orientation blend: 0=wall-derived heading, 1=path-derived heading.',
    )

    return LaunchDescription([
        standoff_arg, tangent_speed_arg, direction_arg, depth_arg, odom_topic_arg,
        goal_topic_arg, path_topic_arg, status_topic_arg, thruster_topic_arg,
        command_topic_arg, safe_command_topic_arg, safety_start_enabled_arg,
        path_influence_arg, path_look_offset_arg, wall_normal_offset_arg,
        path_heading_weight_arg,
        Node(
            package='frontier_slam',
            executable='motion_safety_gate',
            name='motion_safety_gate',
            output='screen',
            parameters=[{
                'command_topic': LaunchConfiguration('command_topic'),
                'output_topic': LaunchConfiguration('safe_command_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'start_enabled': _bool_parameter('safety_start_enabled'),
            }],
        ),
        Node(
            package='frontier_slam',
            executable='heavy_sim_mixer',
            name='heavy_sim_mixer',
            output='screen',
            parameters=[{
                'command_topic': LaunchConfiguration('safe_command_topic'),
                'thruster_topic': LaunchConfiguration('thruster_topic'),
            }],
        ),
        Node(
            package='frontier_slam',
            executable='wall_follower',
            name='wall_follower',
            output='screen',
            parameters=[{
                'standoff_m':     _float_parameter('standoff'),
                'tangent_speed':  _float_parameter('tangent_speed'),
                'direction':      LaunchConfiguration('direction'),
                'depth_setpoint': _float_parameter('depth'),
                'odom_topic':     LaunchConfiguration('odom_topic'),
                'goal_topic':     LaunchConfiguration('goal_topic'),
                'path_topic':     LaunchConfiguration('path_topic'),
                'status_topic':   LaunchConfiguration('status_topic'),
                'command_topic':  LaunchConfiguration('command_topic'),
                'path_influence': _float_parameter('path_influence'),
                'path_look_offset_deg': _float_parameter('path_look_offset_deg'),
                'wall_normal_offset_deg': _float_parameter('wall_normal_offset_deg'),
                'path_heading_weight': _float_parameter('path_heading_weight'),
            }],
        ),
    ])
