"""
Full OctoMap mapping stack: Stonefish + TF + point cloud + octomap_server.

  /cloud_in  (PointCloud2, frame: bluerov2/Dcam)
  TF chain:  world_ned → bluerov2/base_link → bluerov2/Dcam
                ↓
         octomap_server → /octomap_binary            (full 3-D map)
                        → /occupied_cells_vis_array  (RViz MarkerArray)
                        → /projected_map             (2-D occupancy grid)

Thin composition of pointcloud.launch.py and mapper_only.launch.py
(mapper:=octomap). Behaviour is unchanged; launch mapper_only.launch.py by
itself to swap the map backend without dropping the simulator.

Verify with:
  ros2 topic echo /octomap_binary --no-arr
  rviz2 → MarkerArray → /occupied_cells_vis_array
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(
    get_package_share_directory('stonefish_groundtruth_mapping'), 'launch')


def generate_launch_description():
    pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'pointcloud.launch.py'))
    )

    octomap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'mapper_only.launch.py')),
        launch_arguments={'mapper': 'octomap'}.items(),
    )

    return LaunchDescription([pointcloud, octomap])
