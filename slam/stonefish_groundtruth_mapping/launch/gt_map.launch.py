"""
Ground-truth reference map: a second, parallel mapping stack fed from the
EXACT simulator pose (/StoneFish/Odometry), run alongside the SLAM-estimate
map so the two can be visually compared (see bringup/rviz/demo_slam.rviz).

Only included when slam:=slam (see bringup/launch/demo.launch.py) — that's
the only mode where the primary map (built from pose_graph.py's estimate)
can diverge from the truth at all. Under slam:=none the primary map already
IS the ground truth (odom_tf_sync feeds it real pose), so a second copy
here would be redundant.

TF can't hold two transforms for the same frame at once, so this stack runs
its own parallel chain rather than reusing bluerov2/base_link (which
pose_graph.py owns while SLAM is active):

  world_ned -> bluerov2/base_link_gt -> bluerov2/Dcam_gt   (always ground truth)

The depth camera's raw point cloud is identical either way (same simulated
sensor, body-frame data) — only the pose used to place it in world_ned
differs — so cloud_relabel just republishes /cloud_in with its frame_id
swapped to bluerov2/Dcam_gt, and a second octomap_server/tsdf_mapper
instance (topics under /gt/...) integrates it through the ground-truth
chain instead.

Mirrors mapper:=octomap|tsdf so the GT map always matches the belief map's
backend. bringup/demo.launch.py already declares mapper before including
this file, but it's declared here too (DeclareLaunchArgument only fills an
unset value, so this doesn't clobber that) so this file is also launchable
standalone, e.g. for isolated testing.

When the belief map is fed through the sonar noise model (slam:=slam always
sets pointcloud.launch.py's sonar_noise:=true — see that file), /cloud_in is
noisy. The GT map must stay clean, so it defaults to reading /cloud_in_raw
(the pre-noise cloud) instead. Override with gt_cloud_source:=/cloud_in for
standalone use without the noise node running.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mapper_arg = DeclareLaunchArgument(
        'mapper', default_value='octomap', choices=['octomap', 'tsdf'],
        description='Map backend the ground-truth map should mirror',
    )
    gt_cloud_source_arg = DeclareLaunchArgument(
        'gt_cloud_source', default_value='/cloud_in_raw',
        description='Pre-noise cloud topic to feed the ground-truth map from',
    )

    odom_to_tf_gt = Node(
        package='stonefish_groundtruth_mapping',
        executable='odom_tf_sync',
        name='odom_tf_sync_gt',
        output='screen',
        parameters=[{'target_frame': 'bluerov2/base_link_gt'}],
    )

    # Same camera mount offset as tf.launch.py's static_camera_tf, onto the
    # ground-truth base link instead.
    static_camera_tf_gt = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_gt',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.3',
            '--roll', '1.571', '--pitch', '0.0', '--yaw', '1.571',
            '--frame-id', 'bluerov2/base_link_gt',
            '--child-frame-id', 'bluerov2/Dcam_gt',
        ],
    )

    cloud_relabel_gt = Node(
        package='stonefish_groundtruth_mapping',
        executable='cloud_relabel',
        name='cloud_relabel_gt',
        output='screen',
        parameters=[{
            'input_topic':  LaunchConfiguration('gt_cloud_source'),
            'output_topic': '/gt/cloud_in',
            'frame_id':     'bluerov2/Dcam_gt',
        }],
    )

    # namespace='gt' auto-prefixes octomap_server's relative topics
    # (cloud_in, octomap_binary, occupied_cells_vis_array, projected_map)
    # to /gt/... — matches cloud_relabel_gt's output without extra remaps.
    octomap_gt = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_gt',
        namespace='gt',
        output='screen',
        parameters=[{
            'frame_id':               'world_ned',
            'resolution':             0.2,
            'sensor_model/max_range': 15.0,
            'latch':                  True,
        }],
        condition=LaunchConfigurationEquals('mapper', 'octomap'),
    )

    # tsdf_mapper hardcodes absolute topic names (leading '/'), so a
    # namespace wouldn't touch them — every one needs an explicit remap.
    tsdf_gt = Node(
        package='frontier_slam',
        executable='tsdf_mapper',
        name='tsdf_mapper_gt',
        output='screen',
        remappings=[
            ('/cloud_in',                   '/gt/cloud_in'),
            ('/tsdf/surface_cloud',         '/gt/tsdf/surface_cloud'),
            ('/tsdf/surface_normals',       '/gt/tsdf/surface_normals'),
            ('/tsdf/surface_normals_cloud', '/gt/tsdf/surface_normals_cloud'),
            ('/tsdf/voxels',                '/gt/tsdf/voxels'),
        ],
        condition=LaunchConfigurationEquals('mapper', 'tsdf'),
    )

    return LaunchDescription([
        mapper_arg, gt_cloud_source_arg,
        odom_to_tf_gt, static_camera_tf_gt, cloud_relabel_gt,
        octomap_gt, tsdf_gt,
    ])
