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

Thin composition of pointcloud.launch.py and mapper_only.launch.py
(mapper:=tsdf) — parallel structure to octomap.launch.py, same core, different
map backend. Launch mapper_only.launch.py by itself to swap the map backend
without dropping the simulator.

map_rebuild:=true (slam:=slam only; see pose_graph.py, tsdf_mapper.py) makes
this belief-map instance reset+re-integrate from corrected keyframe poses
after a big loop closure. Never touches tsdf_mapper_gt (gt_map.launch.py) —
that instance always keeps enable_rebuild at its default (false).
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

_LAUNCH_DIR = os.path.join(
    get_package_share_directory('stonefish_groundtruth_mapping'), 'launch')


def generate_launch_description():
    map_rebuild_arg = DeclareLaunchArgument(
        'map_rebuild', default_value='false',
        description='Reset+re-integrate this TSDF instance after a big loop closure',
    )

    pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'pointcloud.launch.py'))
    )

    tsdf_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'mapper_only.launch.py')),
        launch_arguments={
            'mapper': 'tsdf',
            'map_rebuild': LaunchConfiguration('map_rebuild'),
        }.items(),
    )

    return LaunchDescription([map_rebuild_arg, pointcloud, tsdf_mapper])
