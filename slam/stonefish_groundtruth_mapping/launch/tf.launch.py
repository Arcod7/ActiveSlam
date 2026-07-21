"""
Stonefish + TF chain.

Starts:
  - Stonefish simulator  (bluerov2 + off_shore_station)  -- core.launch.py
  - odom_to_tf           : /StoneFish/Odometry -> TF(world_ned -> bluerov2/base_link)
  - static camera TF     : bluerov2/base_link  -> bluerov2/Dcam    -- tf_only.launch.py

This file is now a thin composition of core.launch.py (the simulator, which is
expensive to start) and tf_only.launch.py (the TF chain, which is cheap to
restart). Behaviour is unchanged for anything including it; launch the two
halves separately when you need to restart the TF chain without dropping the
simulator.

World path comes from STONEFISH_WORLD_DIR — see core.launch.py.

Verify with:
  ros2 run tf2_tools view_frames
  Expected tree: world_ned -> bluerov2/base_link -> bluerov2/Dcam

use_gt_tf (default true): when false, skips odom_tf_sync so a different node
(the SLAM pose graph — see bringup/launch/demo.launch.py slam:=slam) can be the
sole broadcaster of world_ned -> bluerov2/base_link instead of ground truth.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(
    get_package_share_directory('stonefish_groundtruth_mapping'), 'launch')


def generate_launch_description():
    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'core.launch.py'))
    )

    tf_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'tf_only.launch.py'))
    )

    return LaunchDescription([core, tf_chain])
