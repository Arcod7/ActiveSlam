"""
Frontier-based exploration.

Prerequisites (must already be running):
  ros2 launch stonefish_groundtruth_mapping octomap.launch.py

This adds:
  - frontier_extractor : /projected_map → /frontier_slam/goal + /frontier_slam/path
  - waypoint_controller: /frontier_slam/path + odometry → body command
  - motion_safety_gate: explicit enable + freshness checks
  - heavy_sim_mixer: gated body command → 8 Stonefish Heavy thrusters
  - revisit_planner (revisit:=true only): /slam/dopt → suspends frontier_extractor
    and drives a forced revisit to close a loop when pose uncertainty grows
    (needs a SLAM pose source — see revisit_planner.py)
  - drift_return_scenario (scenario:=drift_return only): a fixed, scripted
    leave-and-return waypoint sequence for watching loop closure fire on
    return (docs/plans/plan.md T1.1) — same suspend/goal interface as
    revisit_planner, see drift_return_scenario.py
  - trajectory_mission (scenario:=trajectory only): follows a launch-settable
    waypoint list or single point, yielding to revisit_planner when it's
    active and resuming where it left off (docs/plans/tracks/
    track_1_trajectory_mission.md) — see trajectory_mission.py

Optional arguments:
  depth       Target depth in NED metres (Z-down, so positive = below surface).
              If omitted, the controller locks the robot's depth on first odometry.
  odom_topic  Odometry topic for both nodes.
  marker_frame  Frame the vehicle arrow is anchored in (world, or the body
              frame on hardware).
              Default: /StoneFish/Odometry (ground truth)
              Noisy:   /StoneFish/Odometry/noisy (requires odom_to_tf_noisy running)
  actuator_backend
              stonefish (default), ardusub_manual, ardusub_local_ned, or none.
              ArduSub never arms or changes mode and requires a dedicated
              BlueOS MAVLink endpoint.
  revisit     Start revisit_planner. Default: false.
  tsdf_frontier_standoff_m
              TSDF-only horizontal distance to hold from a frontier surface,
              measured along its outward normal. Default: 1.0 m.
  projected_map_band_m
              Display only: the Z half-range one /projected_map cell collapses,
              drawn on /frontier_slam/frontier_debug. Match the mapper in use.
              Default: 3.0 (the TSDF mapper's own band).
  scenario    Scripted evaluation scenario: none, drift_return, or trajectory.
              Default: none.
  scenario_out_dx/scenario_out_dy
              Outbound leg offset (m) from the start position for
              scenario:=drift_return. Default: 15.0 / 0.0.
  mission_waypoints
              scenario:=trajectory: flat [x1,y1,z1,x2,y2,z2,...] waypoint
              list in world_ned metres. A single triple is the point case.
  mission_loop
              scenario:=trajectory: repeat mission_waypoints instead of
              finishing after the last one. Default: false.
  scan_style  Scanning motion for waypoint_controller's INIT_SCAN/SCAN/
              GOAL_REACHED states: sweep (default, cable-safe right-then-left,
              docs/plans/plan.md B1 — see scan_sweep.py) or spin (legacy 360°
              rotation, byte-identical to the pre-B1 behaviour).
  scan_sweep_deg
              scan_style:=sweep total sweep width in degrees, mirrored left/
              right about the heading captured when each scan starts.
              Default: 180.0.
  hard_inflation_m/inflation_m/plan_inflation_m
              A* three-zone inflation radii around occupied cells (hard
              block / soft high-cost / planning-margin cost), see
              path_planner.py. Defaults: 0.20 / 0.75 / 1.50 m.

Examples:
  ros2 launch frontier_slam frontier_slam.launch.py
  ros2 launch frontier_slam frontier_slam.launch.py depth:=8.0
  ros2 launch frontier_slam frontier_slam.launch.py odom_topic:=/StoneFish/Odometry/noisy
  ros2 launch frontier_slam frontier_slam.launch.py revisit:=true
  ros2 launch frontier_slam frontier_slam.launch.py scenario:=drift_return

Visualise in RViz2:
  - MarkerArray  /frontier_slam/frontiers  (cyan = candidates, red = active goal)
  - MarkerArray  /frontier_slam/frontier_debug  (why each frontier exists:
    un-inflated boundary cells, the unknown cells bordering them, the Z column
    each 2-D cell collapses, and the TSDF standoff offset)
  - Image        /frontier_slam/planning_dashboard
  - OccupancyGrid /frontier_slam/inflated_map
"""
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _float_parameter(name: str) -> ParameterValue:
    """Resolve a launch argument as a ROS double, including whole numbers."""
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _float_list_parameter(name: str) -> ParameterValue:
    """Resolve a launch argument (a YAML-list string, e.g. '[0.0,0.0,8.0]') as
    a ROS double array. Every element needs an explicit decimal point —
    launch's typed-substitution coercion checks isinstance(x, float), and
    YAML parses a bare '0' as int, which fails that check."""
    return ParameterValue(LaunchConfiguration(name), value_type=List[float])


def _bool_parameter(name: str) -> ParameterValue:
    """Resolve a launch argument as a ROS boolean."""
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def generate_launch_description():
    depth_arg = DeclareLaunchArgument(
        'depth',
        default_value='-1.0',
        description=(
            'Target/cruise depth in NED metres (e.g. depth:=8.0), shared by '
            'frontier_extractor (the Z it anchors its own picks to) and '
            'waypoint_controller (the Z it holds with no active goal). '
            'Omit (or pass depth:=-1) to lock it from the first odometry reading instead.'
        ),
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/StoneFish/Odometry',
        description='Odometry topic for pose. Use /StoneFish/Odometry/noisy for noisy mode.',
    )
    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame',
        default_value='world_ned',
        description=(
            'Frame the vehicle arrow is anchored in. A world frame draws it at '
            'the watched pose; the vehicle body frame draws it at the origin '
            'and lets TF place it, which is what hardware wants.'),
    )
    safety_start_enabled_arg = DeclareLaunchArgument(
        'safety_start_enabled',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Start the motion gate enabled. Keep false for fail-closed operation; '
            'enable explicitly on /motion/enable after completing safety checks.'),
    )
    actuator_backend_arg = DeclareLaunchArgument(
        'actuator_backend',
        default_value='stonefish',
        choices=[
            'stonefish', 'ardusub_manual', 'ardusub_local_ned', 'none'],
        description=(
            'Gated-command consumer. ArduSub choices require pymavlink and a '
            'dedicated BlueOS MAVLink endpoint.'),
    )
    mavlink_url_arg = DeclareLaunchArgument(
        'mavlink_url',
        default_value='udpin:0.0.0.0:14560',
        description='Dedicated BlueOS/pymavlink connection URL.',
    )
    default_ardusub_params = os.path.join(
        get_package_share_directory('frontier_slam'), 'config', 'ardusub.yaml')
    ardusub_params_arg = DeclareLaunchArgument(
        'ardusub_params_file',
        default_value=default_ardusub_params,
        description='ArduSub authority limits and timeout parameters.',
    )
    revisit_arg = DeclareLaunchArgument(
        'revisit',
        default_value='false',
        description='Start revisit_planner (needs a SLAM pose source, e.g. bringup slam:=slam).',
    )
    sigma_allow_xy_arg = DeclareLaunchArgument(
        'sigma_allow_xy_m', default_value='0.045',
        description=(
            'Largest horizontal position sigma the mission tolerates, in metres. '
            'With sigma_allow_yaw_rad it defines D(Sigma_allow), the denominator '
            'of the revisit trigger ratio.'),
    )
    sigma_allow_yaw_arg = DeclareLaunchArgument(
        'sigma_allow_yaw_rad', default_value='0.045',
        description='Largest heading sigma the mission tolerates, in radians.',
    )
    ratio_trigger_arg = DeclareLaunchArgument(
        'ratio_trigger', default_value='1.0',
        description=(
            'Uncertainty ratio U_r = D(Sigma)/D(Sigma_allow) above which exploration '
            'is suspended to drive back and close a loop. 1.0 means "revisit exactly '
            'when the estimate is less certain than the mission allows".'),
    )
    ratio_resume_arg = DeclareLaunchArgument(
        'ratio_resume', default_value='0.5',
        description='Uncertainty ratio below which exploration resumes after a revisit.',
    )
    scenario_arg = DeclareLaunchArgument(
        'scenario',
        default_value='none',
        choices=['none', 'drift_return', 'trajectory'],
        description=(
            'Scripted evaluation scenario: none, drift_return (docs/plans/plan.md T1.1), '
            'or trajectory (docs/plans/tracks/track_1_trajectory_mission.md).'),
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
    mission_waypoints_arg = DeclareLaunchArgument(
        'mission_waypoints',
        default_value='[]',
        description=(
            "trajectory: YAML-list string of flat x,y,z triples in world_ned metres, e.g. "
            "'[0.0,0.0,8.0, 10.0,0.0,8.0, 10.0,10.0,8.0]' — every number needs a decimal "
            "point (a bare '0' parses as int and fails the float-array coercion). A "
            "single triple is the point case. Required — the node rejects an empty or "
            "malformed list at startup."),
    )
    mission_loop_arg = DeclareLaunchArgument(
        'mission_loop',
        default_value='false',
        choices=['true', 'false'],
        description='trajectory: repeat mission_waypoints instead of finishing after the last one.',
    )
    scan_style_arg = DeclareLaunchArgument(
        'scan_style',
        default_value='sweep',
        choices=['sweep', 'spin'],
        description=(
            'Scanning motion for INIT_SCAN/SCAN/GOAL_REACHED: sweep (default, '
            'cable-safe right-then-left) or spin (legacy 360° rotation).'),
    )
    scan_sweep_deg_arg = DeclareLaunchArgument(
        'scan_sweep_deg',
        default_value='180.0',
        description='scan_style:=sweep total sweep width in degrees.',
    )
    motion_arg = DeclareLaunchArgument(
        'motion', default_value='forward',
        choices=['forward', 'walloriented', 'walllooking'],
        description='Selected motion executor for the planner path.',
    )
    speed_factor_arg = DeclareLaunchArgument(
        'speed_factor', default_value='1.0',
        description='Multiplies commanded surge/sway/heave for whichever motion '
                    'executor is active. Same knob as teleop; live-tunable.',
    )
    turn_factor_arg = DeclareLaunchArgument(
        'turn_factor', default_value='1.0',
        description='Multiplies commanded yaw for whichever motion executor is '
                    'active. Same knob as teleop; live-tunable.',
    )
    wall_orientation_offset_arg = DeclareLaunchArgument(
        'wall_orientation_offset_deg', default_value='30.0',
        description='walloriented: fixed yaw offset from the path toward the nearest wall.',
    )
    wall_orientation_lookahead_arg = DeclareLaunchArgument(
        'wall_orientation_lookahead_m', default_value='0.0',
        description=(
            'walloriented: look-heading radius along the A* path in metres; '
            '0 preserves the current-position heading.'),
    )
    tsdf_frontier_standoff_arg = DeclareLaunchArgument(
        'tsdf_frontier_standoff_m', default_value='1.0',
        description='TSDF frontier goal offset from the surface along its outward normal, in metres.',
    )
    projected_map_band_arg = DeclareLaunchArgument(
        'projected_map_band_m', default_value='1.0',
        description=(
            'Display only: the Z half-range one /projected_map cell collapses, '
            'drawn as the band_columns marker on /frontier_slam/frontier_debug. '
            'Must match the mapper publishing the map — pass the same value '
            'here and to octomap.launch.py/tsdf.launch.py, which demo.launch.py '
            'does from its own projected_map_band_m.'),
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', default_value='0.2',
        description=(
            'Display only: sizes the selected-wall highlight walllooking and '
            'walloriented draw on /motion/selected_wall. Must match the '
            'mapper\'s voxel_size, which demo.launch.py passes from its own.'),
    )
    wall_points_topic_arg = DeclareLaunchArgument(
        'wall_points_topic', default_value='/octomap_point_cloud_centers',
        description=(
            'walloriented: XYZ map cloud used to choose left/right. Use '
            '/tsdf/surface_cloud with the TSDF mapper.'),
    )
    wall_standoff_arg = DeclareLaunchArgument(
        'wall_standoff', default_value='1.5',
        description='Wall-follow target standoff in metres.',
    )
    wall_max_surface_dist_arg = DeclareLaunchArgument(
        'wall_max_surface_dist', default_value='8.0',
        description='Furthest a mapped surface can be and still be steered by, in metres.',
    )
    wall_switch_goal_distance_arg = DeclareLaunchArgument(
        'wall_switch_goal_distance', default_value='6.0',
        description='Try a turn-and-select-another-wall search when the goal is within this many metres.',
    )
    wall_switch_scan_angle_arg = DeclareLaunchArgument(
        'wall_switch_scan_angle', default_value='3.14159',
        description='Wall-switch sweep angle in radians (default: 180 degrees).',
    )
    wall_switch_scan_yaw_arg = DeclareLaunchArgument(
        'wall_switch_scan_yaw', default_value='0.08',
        description='Yaw command used for the wall-switch sweep.',
    )
    wall_path_influence_arg = DeclareLaunchArgument(
        'wall_path_influence', default_value='0.70',
        description='Wall-looking travel blend: 0=wall tangent, 1=planned path direction.',
    )
    wall_path_look_offset_arg = DeclareLaunchArgument(
        'wall_path_look_offset_deg', default_value='30.0',
        description='Degrees to turn from the planner bearing toward the selected wall.',
    )
    wall_normal_offset_arg = DeclareLaunchArgument(
        'wall_normal_offset_deg', default_value='0.0',
        description='Degrees to turn from the wall-facing normal toward the planner bearing.',
    )
    wall_path_heading_weight_arg = DeclareLaunchArgument(
        'wall_path_heading_weight', default_value='0.35',
        description='Orientation blend: 0=wall-derived look heading, 1=path-derived look heading.',
    )
    wall_yaw_only_arg = DeclareLaunchArgument(
        'wall_yaw_only', default_value='true', choices=['true', 'false'],
        description=('walllooking: wall guides yaw only; the planner path drives '
                     'translation and wall_standoff/wall_path_influence are ignored.'),
    )
    hard_inflation_arg = DeclareLaunchArgument(
        'hard_inflation_m', default_value='1.00',
        description='A* hard-wall radius around occupied cells (cost = inf).',
    )
    inflation_arg = DeclareLaunchArgument(
        'inflation_m', default_value='1.50',
        description='A* soft-zone radius around occupied cells (high cost, last-resort passage).',
    )
    plan_inflation_arg = DeclareLaunchArgument(
        'plan_inflation_m', default_value='3.00',
        description='A* planning-margin radius around occupied cells (moderate cost, steers paths away).',
    )
    survey_radius_arg = DeclareLaunchArgument(
        'survey_radius_m', default_value='0.0',
        description='Radius (m) of the survey working area; frontier goals outside '
                    'it are not candidates, so exploration stays on the structure '
                    'instead of following open water outward. 0 = unbounded.',
    )
    survey_center_x_arg = DeclareLaunchArgument(
        'survey_center_x', default_value='nan',
        description='Survey-area centre X (NED north, m). nan = the deployment point.',
    )
    survey_center_y_arg = DeclareLaunchArgument(
        'survey_center_y', default_value='nan',
        description='Survey-area centre Y (NED east, m). nan = the deployment point.',
    )
    revisit_schedule_every_m_arg = DeclareLaunchArgument(
        'revisit_schedule_every_m', default_value='0.0',
        description='Revisit every N metres travelled instead of on U_r. '
                    '0 leaves the uncertainty trigger in charge; a positive '
                    'value is the control arm that asks whether triggering on '
                    'uncertainty beats triggering on a clock.')
    revisit_min_closures_arg = DeclareLaunchArgument(
        'revisit_min_closures', default_value='0',
        description='Loop closures required before a revisit ends on closure count '
                    'alone. 0 = not taken into account, leaving the uncertainty ratio '
                    'as the only uncertainty-based exit.',
    )
    arrival_dwell_arg = DeclareLaunchArgument(
        'arrival_dwell_s', default_value='30.0',
        description='Seconds to wait at the revisit target for the uncertainty to '
                    'come back down before giving up on the detour. Counts from '
                    'arrival, so the drive out never shortens it.',
    )
    stall_exit_arg = DeclareLaunchArgument(
        'stall_exit_s', default_value='5.0',
        description='End the dwell once neither the keyframe count nor the closure '
                    'count has moved for this long: a parked vehicle saturates '
                    "pose_graph's per-cell keyframe cap, after which no covariance "
                    'change is possible. 0 disables, leaving arrival_dwell_s.',
    )
    cooldown_arg = DeclareLaunchArgument(
        'cooldown_s', default_value='60.0',
        description='Quiet period after a revisit before the trigger may fire '
                    'again. Sized against mission length, not in the abstract: '
                    'at 60 s a 300 s run with three revisits spends ~60% of '
                    'itself with the trigger suppressed, which compresses any '
                    'difference between trigger policies.',
    )
    dopt_median_window_arg = DeclareLaunchArgument(
        'dopt_median_window', default_value='1',
        description='Median window over /slam/dopt before the trigger reads it. '
                    "iSAM2 recovers each keyframe's marginal from a freshly "
                    'relinearised tree, so the published series dips on ~30% of '
                    'keyframes even with no closures while the underlying '
                    'quantity is monotone between them. 1 disables the filter '
                    'and keeps earlier recorded comparisons comparable.',
    )
    revisit_scan_slowdown_arg = DeclareLaunchArgument(
        'revisit_scan_slowdown', default_value='1.0',
        description='Divide the scan yaw rate by this while a revisit is in progress, '
                    'so the sweep puts more sonar frames on the structure it went back '
                    'to re-observe. 1.0 = off (default; untested in a full run).',
    )
    wall_z_band_arg = DeclareLaunchArgument(
        'wall_z_band_m', default_value='1.5',
        description='Half-thickness (m) of the depth slice wall_oriented uses to pick '
                    'which side to look at. Depth is directly observed, so geometry '
                    'further above or below than this cannot be collided with and '
                    'should not steer the look direction.',
    )
    min_goal_separation_arg = DeclareLaunchArgument(
        'min_goal_separation_m', default_value='0.0',
        description='Minimum distance (m) between consecutive frontier goals, so '
                    'the planner moves on rather than re-picking beside the goal it '
                    'just reached. Waived when no other candidate qualifies. 0 = off.',
    )
    arrival_blacklist_duration_arg = DeclareLaunchArgument(
        'arrival_blacklist_duration_s', default_value='20.0',
        description='Seconds a reached frontier goal stays off the candidate list. '
                    'This is the planner\'s only memory of where it has been: once it '
                    'expires the cluster scores as if never visited. Shorter than the '
                    'travel time to the next goal and a pair of clusters ping-pongs.',
    )
    blacklist_duration_arg = DeclareLaunchArgument(
        'blacklist_duration_s', default_value='30.0',
        description='Seconds a goal abandoned as stuck or unreachable (STUCK, repeated '
                    'A* failure, motion BLOCKED) stays off the candidate list.',
    )
    depth = LaunchConfiguration('depth')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        depth_arg,
        hard_inflation_arg,
        inflation_arg,
        plan_inflation_arg,
        survey_radius_arg,
        survey_center_x_arg,
        survey_center_y_arg,
        min_goal_separation_arg,
        arrival_blacklist_duration_arg,
        blacklist_duration_arg,
        wall_z_band_arg,
        revisit_scan_slowdown_arg,
        revisit_min_closures_arg,
        revisit_schedule_every_m_arg,
        arrival_dwell_arg,
        stall_exit_arg,
        dopt_median_window_arg,
        cooldown_arg,
        odom_topic_arg,
        marker_frame_arg,
        safety_start_enabled_arg,
        actuator_backend_arg,
        mavlink_url_arg,
        ardusub_params_arg,
        revisit_arg,
        sigma_allow_xy_arg,
        sigma_allow_yaw_arg,
        ratio_trigger_arg,
        ratio_resume_arg,
        scenario_arg,
        scenario_out_dx_arg,
        scenario_out_dy_arg,
        mission_waypoints_arg,
        mission_loop_arg,
        scan_style_arg,
        scan_sweep_deg_arg,
        motion_arg,
        speed_factor_arg,
        turn_factor_arg,
        wall_orientation_offset_arg,
        wall_orientation_lookahead_arg,
        tsdf_frontier_standoff_arg,
        projected_map_band_arg,
        voxel_size_arg,
        wall_points_topic_arg,
        wall_standoff_arg,
        wall_max_surface_dist_arg,
        wall_switch_goal_distance_arg,
        wall_switch_scan_angle_arg,
        wall_switch_scan_yaw_arg,
        wall_path_influence_arg,
        wall_path_look_offset_arg,
        wall_normal_offset_arg,
        wall_path_heading_weight_arg,
        wall_yaw_only_arg,
        Node(
            package='frontier_slam',
            executable='motion_safety_gate',
            name='motion_safety_gate',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'marker_frame': LaunchConfiguration('marker_frame'),
                'start_enabled': _bool_parameter('safety_start_enabled'),
            }],
        ),
        Node(
            package='frontier_slam',
            executable='heavy_sim_mixer',
            name='heavy_sim_mixer',
            output='screen',
            condition=LaunchConfigurationEquals(
                'actuator_backend', 'stonefish'),
        ),
        Node(
            package='frontier_slam',
            executable='ardusub_adapter',
            name='ardusub_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('ardusub_params_file'),
                {
                    'backend': 'manual_control',
                    'connection_url': LaunchConfiguration('mavlink_url'),
                },
            ],
            condition=LaunchConfigurationEquals(
                'actuator_backend', 'ardusub_manual'),
        ),
        Node(
            package='frontier_slam',
            executable='ardusub_adapter',
            name='ardusub_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration('ardusub_params_file'),
                {
                    'backend': 'local_ned_velocity',
                    'connection_url': LaunchConfiguration('mavlink_url'),
                },
            ],
            condition=LaunchConfigurationEquals(
                'actuator_backend', 'ardusub_local_ned'),
        ),
        Node(
            package='frontier_slam',
            executable='frontier_extractor',
            name='frontier_extractor',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                # Same value as waypoint_controller's depth_setpoint below —
                # the cruise depth frontier_extractor anchors its own picks
                # to, so goal.point.z is a real depth target either way.
                'depth_setpoint': depth,
                'tsdf_frontier_standoff_m': _float_parameter('tsdf_frontier_standoff_m'),
                'projected_map_band_m': _float_parameter('projected_map_band_m'),
                'hard_inflation_m': _float_parameter('hard_inflation_m'),
                'inflation_m': _float_parameter('inflation_m'),
                'plan_inflation_m': _float_parameter('plan_inflation_m'),
                'survey_radius_m': _float_parameter('survey_radius_m'),
                'survey_center_x': _float_parameter('survey_center_x'),
                'survey_center_y': _float_parameter('survey_center_y'),
                'min_goal_separation_m': _float_parameter('min_goal_separation_m'),
                'arrival_blacklist_duration_s': _float_parameter(
                    'arrival_blacklist_duration_s'),
                'blacklist_duration_s': _float_parameter('blacklist_duration_s'),
            }],
        ),
        Node(
            package='frontier_slam',
            executable='waypoint_controller',
            name='waypoint_controller',
            output='screen',
            parameters=[{
                'depth_setpoint': depth,
                'odom_topic': odom_topic,
                'scan_style': LaunchConfiguration('scan_style'),
                'scan_sweep_deg': _float_parameter('scan_sweep_deg'),
                'revisit_scan_slowdown': _float_parameter('revisit_scan_slowdown'),
                'speed_factor': _float_parameter('speed_factor'),
                'turn_factor': _float_parameter('turn_factor'),
            }],
            condition=LaunchConfigurationEquals('motion', 'forward'),
        ),
        Node(
            package='frontier_slam',
            executable='wall_oriented_controller',
            name='wall_oriented_controller',
            output='screen',
            parameters=[{
                'depth_setpoint': _float_parameter('depth'),
                'odom_topic': odom_topic,
                'look_offset_deg': _float_parameter('wall_orientation_offset_deg'),
                'wall_z_band_m': _float_parameter('wall_z_band_m'),
                # Was never wired: the node kept its 8 m default while the
                # launcher's value only reached wall_looking, so a wall past 8 m
                # selected no side and the viewing offset silently became 0.
                'max_wall_distance_m': _float_parameter('wall_max_surface_dist'),
                'revisit_scan_slowdown': _float_parameter('revisit_scan_slowdown'),
                'lookahead_m': _float_parameter('wall_orientation_lookahead_m'),
                'map_points_topic': LaunchConfiguration('wall_points_topic'),
                'speed_factor': _float_parameter('speed_factor'),
                'turn_factor': _float_parameter('turn_factor'),
                'voxel_size': _float_parameter('voxel_size'),
            }],
            condition=LaunchConfigurationEquals('motion', 'walloriented'),
        ),
        Node(
            package='frontier_slam',
            executable='wall_looking',
            name='wall_looking',
            output='screen',
            parameters=[{
                'depth_setpoint': _float_parameter('depth'),
                'odom_topic': odom_topic,
                'standoff_m': _float_parameter('wall_standoff'),
                'max_surface_dist_m': _float_parameter('wall_max_surface_dist'),
                'switch_goal_distance_m': _float_parameter('wall_switch_goal_distance'),
                'switch_scan_angle_rad': _float_parameter('wall_switch_scan_angle'),
                'switch_scan_yaw': _float_parameter('wall_switch_scan_yaw'),
                'path_influence': _float_parameter('wall_path_influence'),
                'path_look_offset_deg': _float_parameter('wall_path_look_offset_deg'),
                'wall_normal_offset_deg': _float_parameter('wall_normal_offset_deg'),
                'path_heading_weight': _float_parameter('wall_path_heading_weight'),
                'yaw_only': _bool_parameter('wall_yaw_only'),
                'speed_factor': _float_parameter('speed_factor'),
                'turn_factor': _float_parameter('turn_factor'),
                'voxel_size': _float_parameter('voxel_size'),
            }],
            condition=LaunchConfigurationEquals('motion', 'walllooking'),
        ),
        Node(
            package='frontier_slam',
            executable='revisit_planner',
            name='revisit_planner',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'sigma_allow_xy_m': _float_parameter('sigma_allow_xy_m'),
                'sigma_allow_yaw_rad': _float_parameter('sigma_allow_yaw_rad'),
                'ratio_trigger': _float_parameter('ratio_trigger'),
                'ratio_resume': _float_parameter('ratio_resume'),
                'revisit_schedule_every_m': _float_parameter(
                    'revisit_schedule_every_m'),
                'revisit_min_closures': ParameterValue(
                    LaunchConfiguration('revisit_min_closures'), value_type=int),
                'arrival_dwell_s': _float_parameter('arrival_dwell_s'),
                'stall_exit_s': _float_parameter('stall_exit_s'),
                'cooldown_s': _float_parameter('cooldown_s'),
                'dopt_median_window': ParameterValue(
                    LaunchConfiguration('dopt_median_window'), value_type=int),
            }],
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
        Node(
            package='frontier_slam',
            executable='trajectory_mission',
            name='trajectory_mission',
            output='screen',
            parameters=[{
                'odom_topic': odom_topic,
                'mission_waypoints': _float_list_parameter('mission_waypoints'),
                'mission_loop': _bool_parameter('mission_loop'),
            }],
            condition=LaunchConfigurationEquals('scenario', 'trajectory'),
        ),
    ])
