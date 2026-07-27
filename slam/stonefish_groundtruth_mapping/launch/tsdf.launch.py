"""
Full TSDF mapping stack: Stonefish + TF + point cloud + tsdf_mapper.

  /cloud_in  (PointCloud2, frame: bluerov2/Dcam)
  TF chain:  world_ned → bluerov2/base_link → bluerov2/Dcam
                ↓
         tsdf_mapper (VDBFusion) → /tsdf/surface_cloud          (marching-cubes surface)
                                  → /tsdf/surface_normals        (sampled normals, MarkerArray)
                                  → /tsdf/surface_normals_cloud  (points+normals, PointCloud2 —
                                                                  input for wall_looking)
                                  → /tsdf/voxels                 (weight/sign-coded MarkerArray)

tsdf_octomap:=true additionally rebuilds the grid into an octomap::OcTree on
/tsdf/octomap_binary (tsdf_to_octomap) — the octree interface for 3-D frontier
detection and 3-D A*. Kept off /octomap_binary so RViz's OcTree displays stay
octomap_server's alone (see mapper_only.launch.py).

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

    carve_no_return_arg = DeclareLaunchArgument(
        'carve_no_return', default_value='false',
        description='Free the voxels along no-return sonar rays',
    )

    publish_projected_map_arg = DeclareLaunchArgument(
        'publish_projected_map', default_value='false',
        description='Publish a 2-D /projected_map derived from this TSDF grid '
        'for the frontier planner + A*',
    )
    target_depth_m_arg = DeclareLaunchArgument(
        'target_depth_m', default_value='-1.0',
        description='Cruise depth (world_ned Z) the /projected_map band centres on',
    )
    tsdf_octomap_arg = DeclareLaunchArgument(
        'tsdf_octomap', default_value='false',
        description='Rebuild an octomap::OcTree from this TSDF grid and publish '
        'it on /tsdf/octomap_binary (tsdf_to_octomap)',
    )
    voxel_size_arg = DeclareLaunchArgument(
        'voxel_size', default_value='0.2',
        description='TSDF cell size in metres; the truncation band follows at 3x',
    )
    voxel_min_weight_arg = DeclareLaunchArgument(
        'voxel_min_weight', default_value='10.0',
        description='How many times a voxel must be observed to count as a wall',
    )
    voxel_min_solid_confidence_arg = DeclareLaunchArgument(
        'voxel_min_solid_confidence', default_value='0.80',
        description='How far behind the zero crossing a voxel must sit to count '
        'as a wall — 0.5 = at the surface, 1.0 = fully saturated',
    )
    trunc_distance_arg = DeclareLaunchArgument(
        'trunc_distance', default_value='0.0',
        description='Truncation band half-width in metres; 0 = follow voxel_size at 3x',
    )
    space_carving_arg = DeclareLaunchArgument(
        'space_carving', default_value='true',
        description='Free the whole ray from the sensor to the return, not just '
        'the band ahead of the surface',
    )
    directional_tsdf_arg = DeclareLaunchArgument(
        'directional_tsdf', default_value='false',
        description='WIP: one volume per view-direction bin, so a surface seen '
        'from both faces does not average itself away',
    )

    pointcloud = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'pointcloud.launch.py'))
    )

    tsdf_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(_LAUNCH_DIR, 'mapper_only.launch.py')),
        launch_arguments={
            'mapper': 'tsdf',
            'map_rebuild': LaunchConfiguration('map_rebuild'),
            'carve_no_return': LaunchConfiguration('carve_no_return'),
            'publish_projected_map': LaunchConfiguration('publish_projected_map'),
            'target_depth_m': LaunchConfiguration('target_depth_m'),
            'tsdf_octomap': LaunchConfiguration('tsdf_octomap'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'voxel_min_weight': LaunchConfiguration('voxel_min_weight'),
            'voxel_min_solid_confidence': LaunchConfiguration(
                'voxel_min_solid_confidence'),
            'trunc_distance': LaunchConfiguration('trunc_distance'),
            'space_carving': LaunchConfiguration('space_carving'),
            'directional_tsdf': LaunchConfiguration('directional_tsdf'),
        }.items(),
    )

    return LaunchDescription([map_rebuild_arg, carve_no_return_arg,
                              publish_projected_map_arg, target_depth_m_arg,
                              tsdf_octomap_arg, voxel_size_arg,
                              voxel_min_weight_arg, voxel_min_solid_confidence_arg,
                              trunc_distance_arg, space_carving_arg,
                              directional_tsdf_arg,
                              pointcloud, tsdf_mapper])
