"""
Unified demo bring-up: Stonefish + TF + point cloud + mapper + operator mode + RViz.

Usage:
  ros2 launch bringup demo.launch.py                        # teleop + OctoMap (defaults)
  ros2 launch bringup demo.launch.py mode:=frontier          # autonomous frontier exploration
  ros2 launch bringup demo.launch.py mapper:=tsdf            # TSDF surface reconstruction instead of OctoMap
  ros2 launch bringup demo.launch.py mode:=frontier mapper:=tsdf
  ros2 launch bringup demo.launch.py motion:=wallfollow mapper:=tsdf   # wall-normal following
  ros2 launch bringup demo.launch.py slam:=slam noise_profile:=realistic  # SLAM pose + error viz
  ros2 launch bringup demo.launch.py rviz:=false              # headless (e.g. CI, remote box)

`mode`, `motion`, `mapper`, and `slam` are independent axes — the sim+mapping core is
shared, operator mode/map backend/pose source can each be swapped without touching
the others. Exception: motion:=wallfollow consumes /tsdf/surface_normals_cloud, so it
only does something useful with mapper:=tsdf (a LogInfo reminds you at launch).

RViz view is picked automatically (see rviz/demo*.rviz), slam:=slam taking priority:
  slam:=slam    -> demo_slam.rviz: GT vs SLAM vs dead-reckoning paths, a drift arrow
                   + live error/ATE/RPE text HUD, graph edges, covariance ellipsoids,
                   and a second ground-truth-only map (see gt_map.launch.py) overlaid
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
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             LogInfo, SetLaunchConfiguration)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='teleop', choices=['teleop', 'frontier'],
        description='Operator mode: manual keyboard teleop or autonomous frontier exploration',
    )
    motion_arg = DeclareLaunchArgument(
        'motion', default_value='default', choices=['default', 'wallfollow'],
        description='Motion control override: default or wallfollow (needs mapper:=tsdf)',
    )
    mapper_arg = DeclareLaunchArgument(
        'mapper', default_value='octomap', choices=['octomap', 'tsdf'],
        description='Map backend: OctoMap occupancy grid or VDBFusion TSDF',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Launch RViz with the demo view',
    )
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='none', choices=['none', 'slam'],
        description='Pose source: none (ground-truth TF) or slam (GTSAM pose-graph '
                    'correcting simulated pressure/IMU/DVL dead reckoning)',
    )
    noise_profile_arg = DeclareLaunchArgument(
        'noise_profile', default_value='realistic', choices=['ideal', 'realistic', 'degraded'],
        description='Sensor noise profile for slam:=slam (ignored otherwise)',
    )

    # tf.launch.py (included further below via octomap/tsdf -> pointcloud -> tf)
    # declares use_gt_tf with its own default of 'true'; DeclareLaunchArgument only
    # applies a default when the configuration isn't already set, so setting it
    # here — before those includes run — lets slam:=slam suppress the ground-truth
    # broadcaster in favour of pose_graph.py without touching the three
    # intermediate launch files in between.
    set_use_gt_tf = SetLaunchConfiguration(
        'use_gt_tf',
        PythonExpression(["'false' if '", LaunchConfiguration('slam'), "' == 'slam' else 'true'"]),
    )

    # Same propagation trick for the sonar noise model (pointcloud.launch.py):
    # only meaningful once there's a ground-truth map to compare the noisy
    # belief map against, i.e. slam:=slam (see sonar_noise.py, gt_map.launch.py).
    set_sonar_noise = SetLaunchConfiguration(
        'sonar_noise',
        PythonExpression(["'true' if '", LaunchConfiguration('slam'), "' == 'slam' else 'false'"]),
    )

    stonefish_gt_mapping_share = get_package_share_directory('stonefish_groundtruth_mapping')
    frontier_slam_share = get_package_share_directory('frontier_slam')
    slam_backend_share = get_package_share_directory('slam_backend')
    eval_tools_share = get_package_share_directory('eval_tools')
    bringup_share = get_package_share_directory('bringup')

    octomap_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_gt_mapping_share, 'launch', 'octomap.launch.py')
        ),
        condition=LaunchConfigurationEquals('mapper', 'octomap'),
    )

    tsdf_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_gt_mapping_share, 'launch', 'tsdf.launch.py')
        ),
        condition=LaunchConfigurationEquals('mapper', 'tsdf'),
    )

    # Second, parallel map built from the exact simulator pose (not the SLAM
    # estimate) so belief vs. reality can be visually compared in RViz.
    # Only meaningful once pose_graph.py can actually diverge from ground
    # truth, i.e. slam:=slam — see gt_map.launch.py for why this needs its
    # own TF chain rather than reusing bluerov2/base_link.
    gt_map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_gt_mapping_share, 'launch', 'gt_map.launch.py')
        ),
        condition=LaunchConfigurationEquals('slam', 'slam'),
    )

    teleop_hint = LogInfo(
        msg=(
            "mode=teleop: sim + mapper are up. Run teleop yourself in another "
            "terminal (raw keyboard input needs a real TTY, which ros2 launch "
            "can't hand to a Node action):\n"
            "  ros2 run launch_tools my_keyboard"
        ),
        condition=LaunchConfigurationEquals('mode', 'teleop'),
    )

    frontier_exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(frontier_slam_share, 'launch', 'frontier_slam.launch.py')
        ),
        launch_arguments={
            'odom_topic': PythonExpression(
                ["'/slam/odometry' if '", LaunchConfiguration('slam'), "' == 'slam' "
                 "else '/StoneFish/Odometry'"]),
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'frontier'),
    )

    slam_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_backend_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'noise_profile': LaunchConfiguration('noise_profile')}.items(),
        condition=LaunchConfigurationEquals('slam', 'slam'),
    )

    eval_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(eval_tools_share, 'launch', 'eval.launch.py')
        ),
        condition=LaunchConfigurationEquals('slam', 'slam'),
    )

    slam_hint = LogInfo(
        msg=[
            'slam=slam: pose_graph.py is now the sole broadcaster of '
            'world_ned -> bluerov2/base_link (odom_tf_sync is suppressed). '
            'noise_profile=', LaunchConfiguration('noise_profile'),
            ' (also drives the sonar noise model on /cloud_in) — see '
            'eval/runs/<timestamp>/ for ATE/RPE logs. A second, '
            'ground-truth-only map is also running under /gt/... '
            '(demo_slam.rviz overlays it against the belief map).',
        ],
        condition=LaunchConfigurationEquals('slam', 'slam'),
    )

    wall_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(frontier_slam_share, 'launch', 'wall_follow.launch.py')
        ),
        condition=LaunchConfigurationEquals('motion', 'wallfollow'),
    )

    wallfollow_hint = LogInfo(
        msg=(
            'motion=wallfollow: the wall follower consumes /tsdf/surface_normals_cloud '
            '— make sure this launch was started with mapper:=tsdf, otherwise the '
            'robot will scan in place forever.'
        ),
        condition=LaunchConfigurationEquals('motion', 'wallfollow'),
    )

    # Three purpose-built views, picked by mapper/slam rather than hand-edited
    # per run: the plain demo.rviz stays byte-for-byte what it always was
    # (slam:=none mapper:=octomap, the base teleop/frontier demo); mapper:=tsdf
    # swaps in the TSDF-focused view (its OctoMap displays would just show
    # nothing, since octomap_server isn't even launched); slam:=slam takes
    # priority over both because the SLAM view is the only one that adds the
    # ground-truth-vs-estimate comparison (ATE/RPE drift arrow + text HUD,
    # SLAM/dead-reckoning path overlay, covariance ellipsoids, graph edges).
    rviz_config = PythonExpression([
        "'", os.path.join(bringup_share, 'rviz', 'demo_slam.rviz'), "' if '",
        LaunchConfiguration('slam'), "' == 'slam' else ('",
        os.path.join(bringup_share, 'rviz', 'demo_tsdf.rviz'), "' if '",
        LaunchConfiguration('mapper'), "' == 'tsdf' else '",
        os.path.join(bringup_share, 'rviz', 'demo.rviz'), "')",
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        mode_arg, motion_arg, mapper_arg, rviz_arg, slam_arg, noise_profile_arg,
        set_use_gt_tf, set_sonar_noise,
        octomap_stack, tsdf_stack, gt_map_stack,
        slam_stack, eval_stack, slam_hint,
        teleop_hint, frontier_exploration,
        wall_follow, wallfollow_hint,
        rviz,
    ])
