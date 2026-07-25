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

tsdf_octomap:=true adds tsdf_to_octomap alongside tsdf_mapper: the TSDF's
occupied and free voxels are rebuilt into an octomap::OcTree and published on
/tsdf/octomap_binary, giving the TSDF backend the same octree interface 3-D
frontier detection and 3-D A* would consume from octomap_server. It is
deliberately NOT /octomap_binary — that topic is octomap_server's, and RViz's
OcTree displays subscribe to it, so publishing both there overlaid octree
voxels on the TSDF voxels under mapper:=tsdf.

map_rebuild:=true (slam:=slam, mapper:=tsdf only; see pose_graph.py,
tsdf_mapper.py) makes the belief-map instance reset+re-integrate from
corrected keyframe poses after a big loop closure.

`depth` (NED metres, default -1.0 = auto-lock) narrows /projected_map's Z
range to a band centred on that depth (see z_band.py) instead of the
stock full-column projection — skipped when depth is auto-locked, since
the actual depth is unknown at launch time in that case.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from stonefish_groundtruth_mapping.z_band import octomap_z_band_params


def _octomap_node(context, *args, **kwargs):
    depth = float(LaunchConfiguration('depth').perform(context))
    params = {
        'frame_id':               'world_ned',
        'resolution':             0.2,        # 20 cm voxels
        'sensor_model/max_range': 15.0,       # matches Dcam depth_max in .scn
        'latch':                  True,
    }
    params.update(octomap_z_band_params(depth))
    return [Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[params],
        remappings=[
            ('cloud_in', '/cloud_in'),
        ],
        condition=LaunchConfigurationEquals('mapper', 'octomap'),
    )]


def generate_launch_description():
    mapper_arg = DeclareLaunchArgument(
        'mapper', default_value='octomap', choices=['octomap', 'tsdf'],
        description='Map backend: OctoMap occupancy grid or VDBFusion TSDF',
    )
    map_rebuild_arg = DeclareLaunchArgument(
        'map_rebuild', default_value='false',
        description='Reset+re-integrate this TSDF instance after a big loop closure',
    )
    depth_arg = DeclareLaunchArgument(
        'depth', default_value='-1.0',
        description=(
            'Target depth in NED metres, used to centre octomap_server\'s '
            '/projected_map Z-band (mapper:=octomap; the TSDF backend bands its '
            'own projection through target_depth_m). '
            '-1 = auto-lock (unknown at launch time, so the map stays full-column).'),
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
    tsdf_octomap_arg = DeclareLaunchArgument(
        'tsdf_octomap', default_value='false',
        description='TSDF only: rebuild an octomap::OcTree from this TSDF grid '
        'and publish it on /tsdf/octomap_binary (tsdf_to_octomap)',
    )

    # Built through an OpaqueFunction so the Z band can be resolved from `depth`
    # at launch time — see _octomap_node above.
    octomap = OpaqueFunction(function=_octomap_node)

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
            'publish_free_voxels': ParameterValue(
                LaunchConfiguration('tsdf_octomap'), value_type=bool),
        }],
        condition=LaunchConfigurationEquals('mapper', 'tsdf'),
    )

    # resolution mirrors tsdf_mapper's voxel_size default (0.2): the octree
    # cells are the TSDF cells, not a resampling of them.
    tsdf_to_octomap = Node(
        package='tsdf_octomap',
        executable='tsdf_to_octomap',
        name='tsdf_to_octomap',
        output='screen',
        parameters=[{
            'resolution': 0.2,
            'world_frame': 'world_ned',
        }],
        # /octomap_binary belongs to octomap_server (mapper:=octomap) and to
        # RViz's OcTree displays bound to it. Publishing the TSDF-derived tree
        # there too drew octree voxels on top of /tsdf/voxels under
        # mapper:=tsdf, so the planning-facing tree gets its own name.
        remappings=[
            ('/octomap_binary', '/tsdf/octomap_binary'),
            ('/octomap_full', '/tsdf/octomap_full'),
        ],
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('mapper'), "' == 'tsdf' and '",
            LaunchConfiguration('tsdf_octomap'), "'.lower() == 'true'"])),
    )

    return LaunchDescription([mapper_arg, map_rebuild_arg, depth_arg,
                              carve_no_return_arg,
                              publish_projected_map_arg, target_depth_m_arg,
                              projected_map_band_m_arg, projected_map_margin_cells_arg,
                              tsdf_octomap_arg,
                              octomap, tsdf_mapper, tsdf_to_octomap])
