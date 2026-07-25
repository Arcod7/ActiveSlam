"""
Map backend alone: octomap_server or tsdf_mapper, selected by `mapper`.

Consumes /cloud_in and nothing else, so this layer can be restarted — or
switched between backends — without touching the simulator, TF chain or
point cloud below it. octomap.launch.py and tsdf.launch.py both delegate
here, so the original cascading entry points are unchanged.

  mapper:=octomap -> octomap_server -> /octomap_binary            (full 3-D map)
                                     -> /occupied_cells_vis_array (RViz MarkerArray)
                                     -> /projected_map            (2-D occupancy grid)

  mapper:=tsdf    -> tsdf_mapper (VDBFusion) -> /tsdf/surface_cloud
                                              -> /tsdf/surface_normals
                                              -> /tsdf/surface_normals_cloud
                                              -> /tsdf/voxels

map_rebuild:=true (slam:=slam, mapper:=tsdf only; see pose_graph.py,
tsdf_mapper.py) makes the belief-map instance reset+re-integrate from
corrected keyframe poses after a big loop closure.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mapper_arg = DeclareLaunchArgument(
        'mapper', default_value='octomap', choices=['octomap', 'tsdf'],
        description='Map backend: OctoMap occupancy grid or VDBFusion TSDF',
    )
    map_rebuild_arg = DeclareLaunchArgument(
        'map_rebuild', default_value='false',
        description='Reset+re-integrate this TSDF instance after a big loop closure',
    )
    carve_no_return_arg = DeclareLaunchArgument(
        'carve_no_return', default_value='false',
        description='Free the voxels along no-return sonar rays (TSDF only)',
    )
    publish_projected_map_arg = DeclareLaunchArgument(
        'publish_projected_map', default_value='false',
        description='TSDF only: publish a 2-D /projected_map for the frontier '
        'planner + A*, derived straight from this TSDF grid (replaces the '
        'separate octomap_server planning map)',
    )
    target_depth_m_arg = DeclareLaunchArgument(
        'target_depth_m', default_value='-1.0',
        description='Cruise depth (world_ned Z) the /projected_map band centres on; '
        '-1 = lock to the first base_link TF reading',
    )
    projected_map_band_m_arg = DeclareLaunchArgument(
        'projected_map_band_m', default_value='1.0',
        description='Half-thickness (m) of the Z band projected into /projected_map',
    )
    projected_map_margin_cells_arg = DeclareLaunchArgument(
        'projected_map_margin_cells', default_value='10',
        description='Unknown-cell border added around the /projected_map bounding box',
    )

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'frame_id':               'world_ned',
            'resolution':             0.2,        # 20 cm voxels
            'sensor_model/max_range': 15.0,       # matches Dcam depth_max in .scn
            'latch':                  True,
        }],
        remappings=[
            ('cloud_in', '/cloud_in'),
        ],
        condition=LaunchConfigurationEquals('mapper', 'octomap'),
    )

    tsdf_mapper = Node(
        package='frontier_slam',
        executable='tsdf_mapper',
        name='tsdf_mapper',
        output='screen',
        parameters=[{
            'enable_rebuild': LaunchConfiguration('map_rebuild'),
            'carve_no_return': LaunchConfiguration('carve_no_return'),
            'publish_projected_map': ParameterValue(
                LaunchConfiguration('publish_projected_map'), value_type=bool),
            'target_depth_m': ParameterValue(
                LaunchConfiguration('target_depth_m'), value_type=float),
            'projected_map_band_m': ParameterValue(
                LaunchConfiguration('projected_map_band_m'), value_type=float),
            'projected_map_margin_cells': ParameterValue(
                LaunchConfiguration('projected_map_margin_cells'), value_type=int),
        }],
        condition=LaunchConfigurationEquals('mapper', 'tsdf'),
    )

    return LaunchDescription([mapper_arg, map_rebuild_arg, carve_no_return_arg,
                              publish_projected_map_arg, target_depth_m_arg,
                              projected_map_band_m_arg, projected_map_margin_cells_arg,
                              octomap, tsdf_mapper])
