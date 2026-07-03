"""
Full TSDF mapping stack: Stonefish + TF + point cloud + tsdf_mapper.

  /cloud_in  (PointCloud2, frame: bluerov2/Dcam)
  TF chain:  world_ned → bluerov2/base_link → bluerov2/Dcam
                ↓
         tsdf_mapper (VDBFusion) → /tsdf/surface_cloud          (marching-cubes surface)
                                  → /tsdf/surface_normals        (sampled normals, MarkerArray)
                                  → /tsdf/surface_normals_cloud  (points+normals, PointCloud2 —
                                                                  input for wall_follower)
                                  → /tsdf/voxels                 (weight/sign-coded MarkerArray)

Parallel structure to octomap.launch.py — same core, different map backend.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stonefish_groundtruth_mapping'),
                'launch', 'pointcloud.launch.py',
            )
        )
    )

    tsdf_mapper = Node(
        package='frontier_slam',
        executable='tsdf_mapper',
        name='tsdf_mapper',
        output='screen',
    )

    return LaunchDescription([pointcloud, tsdf_mapper])
