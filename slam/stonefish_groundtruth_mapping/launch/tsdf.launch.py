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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_rebuild_arg = DeclareLaunchArgument(
        'map_rebuild', default_value='false',
        description='Reset+re-integrate this TSDF instance after a big loop closure',
    )

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
        parameters=[{'enable_rebuild': LaunchConfiguration('map_rebuild')}],
    )

    return LaunchDescription([map_rebuild_arg, pointcloud, tsdf_mapper])
