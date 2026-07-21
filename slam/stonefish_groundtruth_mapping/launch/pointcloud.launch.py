"""
Depth image → PointCloud2 (includes tf.launch.py: Stonefish + TF chain).

  /sensor_msgs/image_depth  ──► depth_image_proc ──► /cloud_in
  /sensor_msgs/camera_info  ──►┘

Thin composition of tf.launch.py and pointcloud_only.launch.py. Behaviour is
unchanged for anything including it; launch pointcloud_only.launch.py by
itself to restart the cloud/noise layer without dropping the simulator.

NaN replacement (REP 118) is handled inside stonefish_ros2 at publish time.

When sonar_noise:=true (set by demo.launch.py under slam:=slam), a datasheet-
grounded noise model is inserted between depth_image_proc and every consumer,
standing in for a real WaterLinked Sonar 3D-15 (see slam_backend's
sonar_noise.py):

  depth_image_proc ──► /cloud_in_raw ──► sonar_noise ──► /cloud_in

sonar_noise:=false (default, and always under slam:=none) keeps the original
direct wiring so the topology and topic count are unchanged.

Test:
  ros2 topic echo /cloud_in --no-arr
  Expected: header.frame_id = "bluerov2/Dcam", width=257, height=67
  No sphere should appear at the bluerov2/Dcam TF origin in RViz.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(
    get_package_share_directory('stonefish_groundtruth_mapping'), 'launch')


def generate_launch_description():
    tf_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'tf.launch.py'))
    )

    pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_LAUNCH_DIR, 'pointcloud_only.launch.py'))
    )

    return LaunchDescription([tf_chain, pointcloud])
