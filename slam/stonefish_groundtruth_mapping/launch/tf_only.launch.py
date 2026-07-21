"""
TF chain without the simulator: odom_tf_sync + the static camera transform.

  world_ned -> bluerov2/base_link -> bluerov2/Dcam

Split out of tf.launch.py so switching the pose source (ground truth vs. a
SLAM pose graph) only restarts this layer, leaving core.launch.py's
simulator up. tf.launch.py = core.launch.py + this file.

use_gt_tf (default true): when false, skips odom_tf_sync so a different node
(the SLAM pose graph — see bringup/launch/demo.launch.py slam:=slam) can be the
sole broadcaster of world_ned -> bluerov2/base_link instead of ground truth.

Verify with:
  ros2 run tf2_tools view_frames
  Expected tree: world_ned -> bluerov2/base_link -> bluerov2/Dcam
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gt_tf_arg = DeclareLaunchArgument(
        'use_gt_tf', default_value='true',
        description='Broadcast world_ned -> bluerov2/base_link from ground truth. '
                    'Set false when a SLAM pose graph broadcasts that frame instead.',
    )

    odom_to_tf = Node(
        package='stonefish_groundtruth_mapping',
        executable='odom_tf_sync',
        name='odom_tf_sync',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gt_tf')),
    )

    # Camera mount from bluerov2_unphy.scn:
    # <origin rpy="1.571 0.0 1.571" xyz="0.2 0.0 0.3" /> on base_link
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.3',
            '--roll', '1.571', '--pitch', '0.0', '--yaw', '1.571',
            '--frame-id', 'bluerov2/base_link',
            '--child-frame-id', 'bluerov2/Dcam',
        ],
    )

    return LaunchDescription([use_gt_tf_arg, odom_to_tf, static_camera_tf])
