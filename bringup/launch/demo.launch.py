"""
Unified demo bring-up: Stonefish + TF + point cloud + mapper + operator mode + RViz.

Usage:
  ros2 launch bringup demo.launch.py                        # teleop + OctoMap (defaults)
  ros2 launch bringup demo.launch.py mode:=frontier          # autonomous frontier exploration
  ros2 launch bringup demo.launch.py mapper:=tsdf            # TSDF surface reconstruction instead of OctoMap
  ros2 launch bringup demo.launch.py mode:=frontier mapper:=tsdf
  ros2 launch bringup demo.launch.py mode:=frontier motion:=walloriented mapper:=tsdf
                                                                    # forward path with wall-side yaw offset
  ros2 launch bringup demo.launch.py mode:=frontier motion:=walllooking mapper:=tsdf
                                                                    # path following with wall looking
  ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic  # SLAM pose + error viz
  ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic near_cutoff:=1.6  # + clear near-field spray
  ros2 launch bringup demo.launch.py slam:=slam mode:=frontier revisit:=true  # break off exploration to close loops
  ros2 launch bringup demo.launch.py slam:=slam mode:=frontier scenario:=drift_return  # scripted leave-and-return
  ros2 launch bringup demo.launch.py slam:=slam mode:=frontier revisit:=true scenario:=trajectory \
      mission_waypoints:="[0.0,0.0,8.0, 10.0,0.0,8.0, 10.0,10.0,8.0, 0.0,10.0,8.0]"  # follow a reference path, yielding to revisit
  ros2 launch bringup demo.launch.py scene:=target obj_mesh:=shipwreck.obj obj_x:=8 obj_yaw:=30
  ros2 launch bringup demo.launch.py rviz:=false              # headless (e.g. CI, remote box)

`mode`, `motion`, `mapper`, and `slam` are independent axes — the sim+mapping core is
shared, operator mode/map backend/pose source can each be swapped without touching
the others. `mode:=frontier` owns goal/path planning; `motion` chooses its one
executor. motion:=walllooking consumes /tsdf/surface_normals_cloud, so it only does
something useful with mapper:=tsdf (a LogInfo reminds you at launch).

RViz view is picked automatically (see rviz/demo*.rviz), slam:=slam taking priority:
  slam:=slam    -> demo_slam.rviz: GT vs SLAM vs dead-reckoning paths, a drift arrow
                   + a live error/ATE/RPE/D-opt metrics panel docked at the bottom
                   (tools/eval_hud_rviz), graph edges, covariance ellipsoids, and a
                   second ground-truth-only map (see gt_map.launch.py) overlaid
                   against the belief map so map drift/distortion is visible directly
  mapper:=tsdf  -> demo_tsdf.rviz: TSDF surface/voxels instead of OctoMap displays
  otherwise     -> demo.rviz: unchanged base view

Note on mode:=teleop: keyboard_control reads the terminal directly
(termios raw mode), which needs a real TTY — `ros2 launch` doesn't give
its child processes one, so it can't be bundled as a Node action here
(confirmed: it dies with `termios.error: Inappropriate ioctl for device`
when tried). With mode:=teleop this file brings up sim+mapper only and
prints a reminder to run `ros2 run launch_tools my_keyboard` yourself in
another terminal — the standard pattern for ROS2 keyboard teleop.
"""

import os
import sysconfig
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
    get_package_prefix,
)


def _octomap_preload_path():
    """Path to liboctomap.so for RViz's LD_PRELOAD, or '' if it can't be found."""
    try:
        prefix = get_package_prefix("octomap")
    except PackageNotFoundError:
        return ""
    triplet = sysconfig.get_config_var("MULTIARCH") or ""
    candidates = [os.path.join(prefix, "lib", triplet, "liboctomap.so")] if triplet else []
    candidates.append(os.path.join(prefix, "lib", "liboctomap.so"))
    return next((c for c in candidates if os.path.isfile(c)), "")


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="teleop",
        choices=["teleop", "frontier"],
        description="Operator mode: manual keyboard teleop or autonomous frontier exploration",
    )
    safety_start_enabled_arg = DeclareLaunchArgument(
        "safety_start_enabled",
        default_value="false",
        choices=["true", "false"],
        description=(
            "Start the motion safety gate enabled. Keep false and explicitly "
            "enable /motion/enable after checking the scene and controller."),
    )
    motion_arg = DeclareLaunchArgument(
        "motion",
        default_value="default",
        choices=["default", "walloriented", "wall_oriented", "walllooking", "wallfollow"],
        description=("Planner path executor: default/direct, walloriented (OctoMap or TSDF), "
                     "or walllooking (TSDF; wallfollow is a compatibility alias)"),
    )
    wall_orientation_offset_arg = DeclareLaunchArgument(
        "wall_orientation_offset_deg",
        default_value="30.0",
        description="Wall-oriented yaw offset from the planned travel bearing, in degrees",
    )
    wall_orientation_lookahead_arg = DeclareLaunchArgument(
        "wall_orientation_lookahead_m",
        default_value="0.0",
        description=(
            "Wall-oriented lookahead radius along the A* path, in metres; "
            "0 preserves current-position heading"),
    )
    tsdf_frontier_standoff_arg = DeclareLaunchArgument(
        "tsdf_frontier_standoff_m",
        default_value="1.0",
        description=(
            "TSDF frontier goal offset from its surface along the outward wall normal, "
            "in metres"),
    )
    wall_standoff_arg = DeclareLaunchArgument(
        "wall_standoff",
        default_value="1.5",
        description="Wall-follow target standoff in metres",
    )
    wall_switch_goal_distance_arg = DeclareLaunchArgument(
        "wall_switch_goal_distance",
        default_value="6.0",
        description="Turn-and-select-another-wall threshold, in metres from the planner goal",
    )
    wall_switch_scan_angle_arg = DeclareLaunchArgument(
        "wall_switch_scan_angle",
        default_value="3.14159",
        description="Wall-switch sweep angle in radians (default: 180 degrees)",
    )
    wall_switch_scan_yaw_arg = DeclareLaunchArgument(
        "wall_switch_scan_yaw",
        default_value="0.08",
        description="Yaw command used for the wall-switch sweep",
    )
    wall_path_influence_arg = DeclareLaunchArgument(
        "wall_path_influence",
        default_value="0.70",
        description="Wall-looking travel blend: 0=wall tangent, 1=planned path direction",
    )
    wall_path_look_offset_arg = DeclareLaunchArgument(
        "wall_path_look_offset_deg",
        default_value="30.0",
        description="Degrees to turn from the planner bearing toward the selected wall",
    )
    wall_normal_offset_arg = DeclareLaunchArgument(
        "wall_normal_offset_deg",
        default_value="0.0",
        description="Degrees to turn from the wall-facing normal toward the planner bearing",
    )
    wall_path_heading_weight_arg = DeclareLaunchArgument(
        "wall_path_heading_weight",
        default_value="0.35",
        description="Orientation blend: 0=wall-derived heading, 1=path-derived heading",
    )
    mapper_arg = DeclareLaunchArgument(
        "mapper",
        default_value="octomap",
        choices=["octomap", "tsdf"],
        description="Map backend: OctoMap occupancy grid or VDBFusion TSDF",
    )
    tsdf_octomap_arg = DeclareLaunchArgument(
        "tsdf_octomap",
        default_value="true",
        description="mapper:=tsdf only: also rebuild the TSDF grid into an "
        "octomap::OcTree on /octomap_binary (tsdf_to_octomap)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz with the demo view",
    )
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="none",
        choices=["none", "slam"],
        description="Pose source: none (ground-truth TF) or slam (GTSAM pose-graph "
        "correcting simulated pressure/IMU/DVL dead reckoning)",
    )
    noise_profile_arg = DeclareLaunchArgument(
        "noise_profile",
        default_value="realistic",
        choices=[
            "ideal", "sonar_only", "odom_pos_only", "odom_only",
            "realistic", "degraded",
        ],
        description="Sensor noise profile for slam:=slam (ignored otherwise); "
        "sonar_only = realistic sonar noise, near-ideal nav sensors; "
        "odom_pos_only = realistic DVL/pressure, exact orientation and sonar; "
        "odom_only = ground-truth sonar, realistic nav sensors",
    )
    loop_closure_arg = DeclareLaunchArgument(
        "loop_closure",
        default_value="true",
        choices=["true", "false"],
        description="Enable loop-closure detection for slam:=slam (A/B benchmarking switch)",
    )
    noise_seed_arg = DeclareLaunchArgument(
        "noise_seed",
        default_value="-1",
        description="Override the noise profile seed for slam:=slam (-1 = use the profile default)",
    )
    near_cutoff_arg = DeclareLaunchArgument(
        "near_cutoff",
        default_value="-1.0",
        description="Drop noised sonar returns nearer than this (m), over any noise_profile, "
                    "to clear the near-field reverberation spray around the vehicle "
                    "(-1 = profile default, 0 = keep all; e.g. 1.6 cuts the spray).",
    )
    carve_no_return_arg = DeclareLaunchArgument(
        "carve_no_return",
        default_value="false",
        choices=["true", "false"],
        description="TSDF only: free the voxels along no-return sonar rays "
        "instead of leaving open water unknown",
    )
    map_rebuild_arg = DeclareLaunchArgument(
        "map_rebuild",
        default_value="false",
        choices=["true", "false"],
        description="Rebuild the belief map from corrected keyframe poses after a big loop "
        "closure (slam:=slam, TSDF only — see tsdf_mapper.py; unsupported for "
        "mapper:=octomap, see the warning this prints if combined)",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="",
        description="Directory for the eval stack to write TUM/CSV output to "
        "(slam:=slam; default: a timestamped dir under eval/runs/)",
    )
    revisit_arg = DeclareLaunchArgument(
        "revisit",
        default_value="true",
        choices=["true", "false"],
        description="Enable uncertainty-triggered revisit by default for a "
        "SLAM frontier run; suspends exploration to revisit mapped areas when "
        "D-optimality exceeds a threshold (set false to disable).",
    )
    dopt_trigger_arg = DeclareLaunchArgument(
        "dopt_trigger",
        default_value="0.02",
        description="revisit:=true: D-optimality [det(cov_pos)^(1/3)] threshold that "
        "suspends exploration and drives back to close a loop.",
    )
    dopt_resume_arg = DeclareLaunchArgument(
        "dopt_resume",
        default_value="0.01",
        description="revisit:=true: D-optimality threshold below which exploration "
        "resumes after a revisit.",
    )
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="none",
        choices=["none", "drift_return", "trajectory"],
        description="Scripted evaluation scenario: drift_return (docs/plans/plan.md T1.1) "
        "leaves the start position, then returns to it to watch loop closure fire; "
        "trajectory (docs/plans/tracks/track_1_trajectory_mission.md) follows a "
        "waypoint list, yielding to revisit:=true when it triggers (mode:=frontier only)",
    )
    scenario_out_dx_arg = DeclareLaunchArgument(
        "scenario_out_dx",
        default_value="15.0",
        description="drift_return: outbound leg X offset (m) from the captured start position",
    )
    scenario_out_dy_arg = DeclareLaunchArgument(
        "scenario_out_dy",
        default_value="0.0",
        description="drift_return: outbound leg Y offset (m) from the captured start position",
    )
    mission_waypoints_arg = DeclareLaunchArgument(
        "mission_waypoints",
        default_value="[]",
        description="trajectory: YAML-list string of flat x,y,z triples in world_ned metres, "
        "e.g. '[0.0,0.0,8.0, 10.0,0.0,8.0, 10.0,10.0,8.0]' — every number needs a decimal "
        "point (a bare '0' parses as int and fails the float-array coercion). A single "
        "triple is the point case. Required — the node rejects an empty or malformed "
        "list at startup.",
    )
    mission_loop_arg = DeclareLaunchArgument(
        "mission_loop",
        default_value="false",
        choices=["true", "false"],
        description="trajectory: repeat mission_waypoints instead of finishing after the last one",
    )
    scan_style_arg = DeclareLaunchArgument(
        "scan_style",
        default_value="sweep",
        choices=["sweep", "spin"],
        description="Scanning motion for waypoint_controller: sweep (default, "
        "cable-safe right-then-left, docs/plans/plan.md B1) or spin (legacy 360° rotation)",
    )
    scan_sweep_deg_arg = DeclareLaunchArgument(
        "scan_sweep_deg",
        default_value="180.0",
        description="scan_style:=sweep total sweep width in degrees",
    )
    # The core Stonefish launch resolves these against the installed `world`
    # package (or STONEFISH_WORLD_DIR).  Track 2 owns their eventual TUI fields:
    # scene, obj_mesh, obj_x/y/z, obj_scale, and obj_roll/pitch/yaw.
    scene_arg = DeclareLaunchArgument(
        "scene",
        default_value="waterlinked",
        choices=["waterlinked", "target"],
        description="Stonefish scene: baseline waterlinked or lightweight templated target",
    )
    obj_mesh_arg = DeclareLaunchArgument(
        "obj_mesh", default_value="pipe",
        description="target object: pipe or an .obj/.stl filename from world data/obj",
    )
    obj_x_arg = DeclareLaunchArgument(
        "obj_x", default_value="8.0",
        description="target object X position in world_ned (m)",
    )
    obj_y_arg = DeclareLaunchArgument(
        "obj_y", default_value="-2.0",
        description="target object Y position in world_ned (m)",
    )
    obj_z_arg = DeclareLaunchArgument(
        "obj_z", default_value="8.0",
        description="target object Z position in world_ned (m)",
    )
    obj_scale_arg = DeclareLaunchArgument(
        "obj_scale", default_value="1.0",
        description="target object uniform scale (> 0)",
    )
    obj_roll_arg = DeclareLaunchArgument(
        "obj_roll", default_value="0.0",
        description="target object roll (degrees)",
    )
    obj_pitch_arg = DeclareLaunchArgument(
        "obj_pitch", default_value="0.0",
        description="target object pitch (degrees)",
    )
    obj_yaw_arg = DeclareLaunchArgument(
        "obj_yaw", default_value="0.0",
        description="target object yaw (degrees)",
    )
    robot_x_arg = DeclareLaunchArgument("robot_x", default_value="0.0",
                                        description="robot X position in world_ned (m)")
    robot_y_arg = DeclareLaunchArgument("robot_y", default_value="0.0",
                                        description="robot Y position in world_ned (m)")
    robot_z_arg = DeclareLaunchArgument("robot_z", default_value="8.0",
                                        description="robot Z position in world_ned (m)")
    robot_roll_arg = DeclareLaunchArgument("robot_roll", default_value="0.0",
                                           description="robot roll (degrees)")
    robot_pitch_arg = DeclareLaunchArgument("robot_pitch", default_value="0.0",
                                            description="robot pitch (degrees)")
    robot_yaw_arg = DeclareLaunchArgument("robot_yaw", default_value="0.0",
                                          description="robot yaw (degrees)")
    depth_arg = DeclareLaunchArgument(
        "depth", default_value="8.0",
        description="fixed autonomous depth target in NED metres (positive = below surface)",
    )
    speed_factor_arg = DeclareLaunchArgument(
        "speed_factor", default_value="1.0",
        description="Multiplies commanded surge/sway/heave in both teleop and "
        "frontier motion (whichever executor is active). Live-tunable.",
    )
    turn_factor_arg = DeclareLaunchArgument(
        "turn_factor", default_value="1.0",
        description="Multiplies commanded yaw in both teleop and frontier motion "
        "(whichever executor is active). Live-tunable.",
    )

    # tf.launch.py (included further below via octomap/tsdf -> pointcloud -> tf)
    # declares use_gt_tf with its own default of 'true'; DeclareLaunchArgument only
    # applies a default when the configuration isn't already set, so setting it
    # here — before those includes run — lets slam:=slam suppress the ground-truth
    # broadcaster in favour of pose_graph.py without touching the three
    # intermediate launch files in between.
    set_use_gt_tf = SetLaunchConfiguration(
        "use_gt_tf",
        PythonExpression(
            ["'false' if '", LaunchConfiguration("slam"), "' == 'slam' else 'true'"]
        ),
    )

    # Same propagation trick for the sonar noise model (pointcloud.launch.py):
    # only meaningful once there's a ground-truth map to compare the noisy
    # belief map against, i.e. slam:=slam (see sonar_noise.py, gt_map.launch.py).
    set_sonar_noise = SetLaunchConfiguration(
        "sonar_noise",
        PythonExpression(
            ["'true' if '", LaunchConfiguration("slam"), "' == 'slam' else 'false'"]
        ),
    )

    stonefish_gt_mapping_share = get_package_share_directory(
        "stonefish_groundtruth_mapping"
    )
    frontier_slam_share = get_package_share_directory("frontier_slam")
    slam_backend_share = get_package_share_directory("slam_backend")
    eval_tools_share = get_package_share_directory("eval_tools")
    bringup_share = get_package_share_directory("bringup")

    octomap_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_gt_mapping_share, "launch", "octomap.launch.py")
        ),
        condition=LaunchConfigurationEquals("mapper", "octomap"),
    )

    # Under mode:=frontier the TSDF mapper derives /projected_map itself (a thin
    # depth band projected straight from its own grid), so the frontier planner
    # and A* share the belief map instead of a second, independent
    # octomap_server. target_depth_m centres the projection band on the cruise
    # depth (see tsdf_mapper.py / demo depth arg).
    tsdf_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_gt_mapping_share, "launch", "tsdf.launch.py")
        ),
        launch_arguments={
            "publish_projected_map": PythonExpression(
                ["'true' if '", LaunchConfiguration("mode"),
                 "' == 'frontier' else 'false'"]
            ),
            "target_depth_m": LaunchConfiguration("depth"),
            "tsdf_octomap": LaunchConfiguration("tsdf_octomap"),
        }.items(),
        condition=LaunchConfigurationEquals("mapper", "tsdf"),
    )

    tsdf_planning_map_hint = LogInfo(
        msg=(
            "mode=frontier mapper=tsdf: tsdf_mapper derives /projected_map from "
            "its own grid (banded around the target depth) — no separate "
            "octomap_server planning map."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("mode"),
                    "' == 'frontier' and '",
                    LaunchConfiguration("mapper"),
                    "' == 'tsdf'",
                ]
            )
        ),
    )

    # Second, parallel map built from the exact simulator pose (not the SLAM
    # estimate) so belief vs. reality can be visually compared in RViz.
    # Only meaningful once pose_graph.py can actually diverge from ground
    # truth, i.e. slam:=slam — see gt_map.launch.py for why this needs its
    # own TF chain rather than reusing bluerov2/base_link.
    gt_map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_gt_mapping_share, "launch", "gt_map.launch.py")
        ),
        condition=LaunchConfigurationEquals("slam", "slam"),
    )

    teleop_hint = LogInfo(
        msg=(
            "mode=teleop: sim + mapper are up. Run teleop yourself in another "
            "terminal (raw keyboard input needs a real TTY, which ros2 launch "
            "can't hand to a Node action):\n"
            "  ros2 run launch_tools my_keyboard\n"
            "Then use the Motion Safety panel in RViz to enable motion only "
            "when safe (the panel does not arm/disarm ArduSub)."
        ),
        condition=LaunchConfigurationEquals("mode", "teleop"),
    )

    teleop_safety_gate = Node(
        package="frontier_slam",
        executable="motion_safety_gate",
        name="motion_safety_gate",
        output="screen",
        parameters=[{
            "odom_topic": PythonExpression(
                [
                    "'/slam/odometry' if '",
                    LaunchConfiguration("slam"),
                    "' == 'slam' else '/StoneFish/Odometry'",
                ]
            ),
            "start_enabled": ParameterValue(
                LaunchConfiguration("safety_start_enabled"), value_type=bool),
        }],
        condition=LaunchConfigurationEquals("mode", "teleop"),
    )

    teleop_sim_mixer = Node(
        package="frontier_slam",
        executable="heavy_sim_mixer",
        name="heavy_sim_mixer",
        output="screen",
        condition=LaunchConfigurationEquals("mode", "teleop"),
    )

    frontier_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(frontier_slam_share, "launch", "frontier_slam.launch.py")
        ),
        launch_arguments={
            "odom_topic": PythonExpression(
                [
                    "'/slam/odometry' if '",
                    LaunchConfiguration("slam"),
                    "' == 'slam' else '/StoneFish/Odometry'",
                ]
            ),
            "revisit": PythonExpression(
                [
                    "'true' if '",
                    LaunchConfiguration("revisit"),
                    "' == 'true' and '",
                    LaunchConfiguration("slam"),
                    "' == 'slam' else 'false'",
                ]
            ),
            "dopt_trigger": LaunchConfiguration("dopt_trigger"),
            "dopt_resume": LaunchConfiguration("dopt_resume"),
            "scenario": LaunchConfiguration("scenario"),
            "scenario_out_dx": LaunchConfiguration("scenario_out_dx"),
            "scenario_out_dy": LaunchConfiguration("scenario_out_dy"),
            "mission_waypoints": LaunchConfiguration("mission_waypoints"),
            "mission_loop": LaunchConfiguration("mission_loop"),
            "scan_style": LaunchConfiguration("scan_style"),
            "scan_sweep_deg": LaunchConfiguration("scan_sweep_deg"),
            "depth": LaunchConfiguration("depth"),
            "speed_factor": LaunchConfiguration("speed_factor"),
            "turn_factor": LaunchConfiguration("turn_factor"),
            "safety_start_enabled": LaunchConfiguration("safety_start_enabled"),
            "motion": PythonExpression(
                [
                    "'walllooking' if '",
                    LaunchConfiguration("motion"),
                    "' in ('walllooking', 'wallfollow') else ('walloriented' if '",
                    LaunchConfiguration("motion"),
                    "' in ('walloriented', 'wall_oriented') else 'forward')",
                ]
            ),
            "wall_orientation_offset_deg": LaunchConfiguration(
                "wall_orientation_offset_deg"),
            "wall_orientation_lookahead_m": LaunchConfiguration(
                "wall_orientation_lookahead_m"),
            "tsdf_frontier_standoff_m": LaunchConfiguration(
                "tsdf_frontier_standoff_m"),
            "wall_points_topic": PythonExpression(
                [
                    "'/tsdf/surface_cloud' if '",
                    LaunchConfiguration("mapper"),
                    "' == 'tsdf' else '/octomap_point_cloud_centers'",
                ]
            ),
            "wall_standoff": LaunchConfiguration("wall_standoff"),
            "wall_switch_goal_distance": LaunchConfiguration("wall_switch_goal_distance"),
            "wall_switch_scan_angle": LaunchConfiguration("wall_switch_scan_angle"),
            "wall_switch_scan_yaw": LaunchConfiguration("wall_switch_scan_yaw"),
            "wall_path_influence": LaunchConfiguration("wall_path_influence"),
            "wall_path_look_offset_deg": LaunchConfiguration("wall_path_look_offset_deg"),
            "wall_normal_offset_deg": LaunchConfiguration("wall_normal_offset_deg"),
            "wall_path_heading_weight": LaunchConfiguration("wall_path_heading_weight"),
        }.items(),
        condition=LaunchConfigurationEquals("mode", "frontier"),
    )

    slam_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_backend_share, "launch", "slam.launch.py")
        ),
        launch_arguments={
            "noise_profile": LaunchConfiguration("noise_profile"),
            "loop_closure": LaunchConfiguration("loop_closure"),
            "noise_seed": LaunchConfiguration("noise_seed"),
            "map_rebuild": LaunchConfiguration("map_rebuild"),
            "initial_x": LaunchConfiguration("robot_x"),
            "initial_y": LaunchConfiguration("robot_y"),
        }.items(),
        condition=LaunchConfigurationEquals("slam", "slam"),
    )

    eval_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eval_tools_share, "launch", "eval.launch.py")
        ),
        launch_arguments={
            "output_dir": LaunchConfiguration("output_dir"),
            "mapper": LaunchConfiguration("mapper"),
        }.items(),
        condition=LaunchConfigurationEquals("slam", "slam"),
    )

    slam_hint = LogInfo(
        msg=[
            "slam=slam: pose_graph.py is now the sole broadcaster of "
            "world_ned -> bluerov2/base_link (odom_tf_sync is suppressed). "
            "noise_profile=",
            LaunchConfiguration("noise_profile"),
            " (also drives the sonar noise model on /cloud_in) — see "
            "eval/runs/<timestamp>/ for ATE/RPE logs. A second, "
            "ground-truth-only map is also running under /gt/... "
            "(demo_slam.rviz overlays it against the belief map).",
        ],
        condition=LaunchConfigurationEquals("slam", "slam"),
    )

    walllooking_hint = LogInfo(
        msg=(
            "motion=walllooking: path following is the primary motion policy; TSDF walls "
            "only influence travel and viewing orientation. It is the only controller and "
            "consumes the planner path plus /tsdf/surface_normals_cloud. Use mapper:=tsdf; "
            "if no wall can support the requested path, it reports BLOCKED to the planner."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("motion"),
                    "' in ('walllooking', 'wallfollow') and '",
                    LaunchConfiguration("mode"),
                    "' == 'frontier'",
                ]
            )
        ),
    )

    walloriented_hint = LogInfo(
        msg=[
            "motion=walloriented: following the planner path while looking ",
            LaunchConfiguration("wall_orientation_offset_deg"),
            " degrees toward the nearest ",
            LaunchConfiguration("mapper"),
            " surface. Missing/stale map points fall back to direct forward orientation.",
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("motion"),
                    "' in ('walloriented', 'wall_oriented') and '",
                    LaunchConfiguration("mode"),
                    "' == 'frontier'",
                ]
            )
        ),
    )

    revisit_needs_slam_warning = LogInfo(
        msg=(
            "revisit=true requires slam:=slam (the trigger consumes /slam/dopt, only "
            "published by the SLAM pose graph) — the revisit planner will not be started."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("revisit"),
                    "' == 'true' and '",
                    LaunchConfiguration("slam"),
                    "' == 'none'",
                ]
            )
        ),
    )

    scenario_needs_frontier_warning = LogInfo(
        msg=(
            "scenario requires mode:=frontier (it drives via "
            "frontier_extractor/waypoint_controller) — the scenario node will not be started."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("scenario"),
                    "' != 'none' and '",
                    LaunchConfiguration("mode"),
                    "' != 'frontier'",
                ]
            )
        ),
    )

    map_rebuild_octomap_warning = LogInfo(
        msg=(
            "map_rebuild=true has no effect with mapper:=octomap: octomap_server raycasts "
            "free space from a TF-at-stamp sensor origin, which replaying corrected "
            "keyframe scans cannot reproduce cleanly. Map rebuild is TSDF-only "
            "(tsdf_mapper.py) — use mapper:=tsdf to actually rebuild the belief map."
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("map_rebuild"),
                    "' == 'true' and '",
                    LaunchConfiguration("mapper"),
                    "' == 'octomap'",
                ]
            )
        ),
    )

    # Three purpose-built views, picked by mapper/slam rather than hand-edited
    # per run: the plain demo.rviz stays byte-for-byte what it always was
    # (slam:=none mapper:=octomap, the base teleop/frontier demo); mapper:=tsdf
    # swaps in the TSDF-focused view (its OctoMap displays would just show
    # nothing, since octomap_server isn't even launched); slam:=slam takes
    # priority over both because the SLAM view is the only one that adds the
    # ground-truth-vs-estimate comparison (ATE/RPE drift arrow + text HUD,
    # SLAM/dead-reckoning path overlay, covariance ellipsoids, graph edges).
    rviz_config = PythonExpression(
        [
            "'",
            os.path.join(bringup_share, "rviz", "demo_slam.rviz"),
            "' if '",
            LaunchConfiguration("slam"),
            "' == 'slam' else ('",
            os.path.join(bringup_share, "rviz", "demo_tsdf.rviz"),
            "' if '",
            LaunchConfiguration("mapper"),
            "' == 'tsdf' else '",
            os.path.join(bringup_share, "rviz", "demo.rviz"),
            "')",
        ]
    )

    # liboctomap lives under a Debian multiarch triplet directory, so derive the
    # triplet instead of naming one architecture. Falls back to the unsuffixed
    # lib/ path, then to no preload at all if the library isn't where expected.
    octomap_lib_preload = _octomap_preload_path()

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        additional_env={"LD_PRELOAD": octomap_lib_preload},
    )

    return LaunchDescription(
        [
            mode_arg,
            safety_start_enabled_arg,
            motion_arg,
            wall_orientation_offset_arg,
            wall_orientation_lookahead_arg,
            tsdf_frontier_standoff_arg,
            wall_standoff_arg,
            wall_switch_goal_distance_arg,
            wall_switch_scan_angle_arg,
            wall_switch_scan_yaw_arg,
            wall_path_influence_arg,
            wall_path_look_offset_arg,
            wall_normal_offset_arg,
            wall_path_heading_weight_arg,
            mapper_arg,
            tsdf_octomap_arg,
            rviz_arg,
            slam_arg,
            noise_profile_arg,
            loop_closure_arg,
            noise_seed_arg,
            near_cutoff_arg,
            map_rebuild_arg,
            carve_no_return_arg,
            output_dir_arg,
            revisit_arg,
            dopt_trigger_arg,
            dopt_resume_arg,
            scenario_arg,
            scenario_out_dx_arg,
            scenario_out_dy_arg,
            mission_waypoints_arg,
            mission_loop_arg,
            scan_style_arg,
            scan_sweep_deg_arg,
            scene_arg,
            obj_mesh_arg,
            obj_x_arg,
            obj_y_arg,
            obj_z_arg,
            obj_scale_arg,
            obj_roll_arg,
            obj_pitch_arg,
            obj_yaw_arg,
            robot_x_arg,
            robot_y_arg,
            robot_z_arg,
            robot_roll_arg,
            robot_pitch_arg,
            robot_yaw_arg,
            depth_arg,
            speed_factor_arg,
            turn_factor_arg,
            set_use_gt_tf,
            set_sonar_noise,
            # revisit_needs_slam_warning/scenario_needs_frontier_warning read top-level
            # configs and must be visited BEFORE frontier_exploration: that include
            # temporarily rescopes those configs (Push/Pop) for its own sub-launch, and
            # the Pop isn't guaranteed to have completed by the time a later list entry
            # is visited.
            revisit_needs_slam_warning,
            scenario_needs_frontier_warning,
            octomap_stack,
            tsdf_stack,
            tsdf_planning_map_hint,
            gt_map_stack,
            slam_stack,
            eval_stack,
            slam_hint,
            teleop_safety_gate,
            teleop_sim_mixer,
            teleop_hint,
            frontier_exploration,
            walloriented_hint,
            walllooking_hint,
            map_rebuild_octomap_warning,
            rviz,
        ]
    )
