"""
Stonefish + TF chain.

Starts:
  - Stonefish simulator  (bluerov2 + off_shore_station)
  - odom_to_tf           : /StoneFish/Odometry → TF(world_ned → bluerov2/base_link)
  - static camera TF     : bluerov2/base_link  → bluerov2/Dcam

World path is read from the STONEFISH_WORLD_DIR environment variable.
Default: the installed `world` package's share directory (reliable
regardless of symlink-install vs. copy-install — colcon doesn't guarantee
every data_files entry becomes a symlink on every rebuild, so resolving
via __file__ is not safe; get_package_share_directory() is what
ament_index is for).
Override before launching:
  export STONEFISH_WORLD_DIR=/path/to/world

Verify with:
  ros2 run tf2_tools view_frames
  Expected tree: world_ned → bluerov2/base_link → bluerov2/Dcam

use_gt_tf (default true): when false, skips odom_tf_sync so a different node
(the SLAM pose graph — see bringup/launch/demo.launch.py slam:=slam) can be the
sole broadcaster of world_ned -> bluerov2/base_link instead of ground truth.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_WORLD_DIR = os.environ.get('STONEFISH_WORLD_DIR', get_package_share_directory('world'))
WORLD_DATA = os.path.join(_WORLD_DIR, 'data')
SCENARIO   = os.path.join(_WORLD_DIR, 'scnenario', 'waterlinked.scn')


def generate_launch_description():
    use_gt_tf_arg = DeclareLaunchArgument(
        'use_gt_tf', default_value='true',
        description='Broadcast world_ned -> bluerov2/base_link from ground truth. '
                    'Set false when a SLAM pose graph broadcasts that frame instead.',
    )

    stonefish = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[WORLD_DATA, SCENARIO, '300', '1200', '900', 'medium'],
        output='screen',
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

    return LaunchDescription([use_gt_tf_arg, stonefish, odom_to_tf, static_camera_tf])
