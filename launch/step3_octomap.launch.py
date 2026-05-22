"""
Step 3 — OctoMap ground-truth map.

Adds octomap_server on top of Step 2.

  /cloud_in  (PointCloud2, frame: bluerov2/Dcam)
  TF chain:  world_ned → bluerov2/base_link → bluerov2/Dcam
                ↓
         octomap_server → /octomap_binary            (full 3-D map)
                        → /occupied_cells_vis_array  (RViz MarkerArray)
                        → /projected_map             (2-D occupancy grid)

Verify with:
  ros2 topic echo /octomap_binary --no-arr
  rviz2 → MarkerArray → /occupied_cells_vis_array
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    step2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stonefish_groundtruth_mapping'),
                'launch', 'step2_pointcloud.launch.py',
            )
        )
    )

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id':               'world_ned',
            'resolution':             0.1,        # 10 cm voxels
            'sensor_model/max_range': 15.0,       # matches Dcam depth_max in .scn
        }],
        remappings=[
            ('cloud_in', '/cloud_in'),
        ],
    )

    return LaunchDescription([step2, octomap])
