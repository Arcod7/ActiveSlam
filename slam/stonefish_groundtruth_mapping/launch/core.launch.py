"""
Stonefish simulator alone — the one layer that is expensive to start.

Split out of tf.launch.py so the layers above it (TF, point cloud, mapper,
planner, SLAM) can be restarted independently while the simulator keeps
running. tf.launch.py still includes this file, so the original cascade
(tf -> pointcloud -> octomap/tsdf) behaves exactly as before.

World path is read from the STONEFISH_WORLD_DIR environment variable.
Default: the installed `world` package's share directory (reliable
regardless of symlink-install vs. copy-install — colcon doesn't guarantee
every data_files entry becomes a symlink on every rebuild, so resolving
via __file__ is not safe; get_package_share_directory() is what
ament_index is for).
Override before launching:
  export STONEFISH_WORLD_DIR=/path/to/world

scene:=waterlinked opens the original scenario untouched.  scene:=target
renders the installed object template under /tmp with obj_mesh and the obj_*
launch arguments before Stonefish starts.
"""
import hashlib
import os
import math
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from string import Template
from xml.sax.saxutils import escape

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

_WORLD_DIR = os.environ.get('STONEFISH_WORLD_DIR', get_package_share_directory('world'))
WORLD_DATA = os.path.join(_WORLD_DIR, 'data')
SCENARIO_DIR = os.path.join(_WORLD_DIR, 'scenario')
DEFAULT_SCENE = 'waterlinked'
TARGET_SCENE = 'target'
SCENES = (DEFAULT_SCENE, TARGET_SCENE)
BUILTIN_PIPE = 'pipe'
DEFAULT_ROBOT_POSE = ((0.0, 0.0, 8.0), (0.0, 0.0, 0.0))
# Debug/convenience only: scales the simulated thruster RPM ceiling
# (max_setpoint) past the datasheet-grounded default for fast repositioning
# between runs. Not a physically realistic BlueROV2/T200 value.
THRUST_BOOST_MULTIPLIER = 2.5


def _finite_float(context, name, *, positive=False):
    """Resolve and validate a numeric launch argument before rendering XML."""
    raw = LaunchConfiguration(name).perform(context)
    try:
        value = float(raw)
    except ValueError as exc:
        raise RuntimeError(f'{name} must be a number, got {raw!r}') from exc
    if not math.isfinite(value) or (positive and value <= 0.0):
        requirement = 'a finite positive number' if positive else 'a finite number'
        raise RuntimeError(f'{name} must be {requirement}, got {raw!r}')
    return value


def _selected_mesh(context):
    """Return the user-selected filename after constraining it to data/obj."""
    mesh = LaunchConfiguration('obj_mesh').perform(context)
    if mesh == BUILTIN_PIPE:
        return mesh
    obj_dir = Path(WORLD_DATA, 'obj')
    available = {path.name for path in obj_dir.iterdir()
                 if path.is_file() and path.suffix.lower() in ('.obj', '.stl')}
    if mesh not in available:
        raise RuntimeError(
            f'obj_mesh must be {BUILTIN_PIPE!r} or an .obj/.stl file in '
            f'{obj_dir}; got {mesh!r}')
    return mesh


def _robot_pose(context):
    """Resolve robot launch pose; TUI angles are degrees, Stonefish uses rad."""
    xyz = tuple(_finite_float(context, name) for name in
                ('robot_x', 'robot_y', 'robot_z'))
    rpy = tuple(math.radians(_finite_float(context, name)) for name in
                ('robot_roll', 'robot_pitch', 'robot_yaw'))
    return xyz, rpy


def _scene_output_path(prefix, content):
    scene_id = hashlib.sha256(content.encode('utf-8')).hexdigest()[:16]
    output_dir = Path(tempfile.gettempdir(), 'active_slam_stonefish_scenes')
    output_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    output_path = output_dir / f'{prefix}-{scene_id}.scn'
    output_path.write_text(content, encoding='utf-8')
    return str(output_path)


def _render_robot_scene(robot_pose, thrust_boost=False):
    """Clone the robot description with a launch-time pose, never mutating it."""
    source = Path(WORLD_DATA, 'robot', 'bluerov2_unphy.scn')
    try:
        root = ET.parse(source).getroot()
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(f'Cannot read robot scenario {source}') from exc
    transform = root.find('.//robot/world_transform')
    if transform is None:
        raise RuntimeError(f'Robot scenario {source} has no world_transform')
    xyz, rpy = robot_pose
    transform.set('xyz', ' '.join(format(value, '.9g') for value in xyz))
    transform.set('rpy', ' '.join(format(value, '.9g') for value in rpy))
    if thrust_boost:
        for specs in root.findall('.//specs[@max_setpoint]'):
            boosted = float(specs.get('max_setpoint')) * THRUST_BOOST_MULTIPLIER
            specs.set('max_setpoint', format(boosted, '.9g'))
    rendered = ET.tostring(root, encoding='unicode', xml_declaration=True)
    return _scene_output_path('bluerov2', rendered)


def _target_body(mesh, values, scale):
    """XML for one static object, with only whitelisted mesh paths allowed."""
    if mesh == BUILTIN_PIPE:
        radius = format(0.35 * scale, '.9g')
        height = format(4.0 * scale, '.9g')
        return f'''    <static name="SonarTarget" type="cylinder">
        <dimensions radius="{radius}" height="{height}"/>
        <material name="Aluminium"/>
        <look name="Gray"/>
        <world_transform xyz="{values['OBJ_X']} {values['OBJ_Y']} {values['OBJ_Z']}"
                         rpy="{values['OBJ_ROLL']} {values['OBJ_PITCH']} {values['OBJ_YAW']}"/>
    </static>'''

    # Meshes are intentionally unmodified: the operator owns their scale,
    # placement and orientation through obj_scale and obj_{x,y,z,roll,pitch,yaw}.
    mesh_scale = format(scale, '.9g')
    origin = '0.0 0.0 0.0'
    safe_mesh = escape(mesh, {'"': '&quot;'})
    return f'''    <static name="SonarTarget" type="model">
        <physical>
            <mesh filename="obj/{safe_mesh}" scale="{mesh_scale}" convex="false"/>
            <origin rpy="{origin}" xyz="0.0 0.0 0.0"/>
        </physical>
        <visual>
            <mesh filename="obj/{safe_mesh}" scale="{mesh_scale}"/>
            <origin rpy="{origin}" xyz="0.0 0.0 0.0"/>
        </visual>
        <material name="Rock"/>
        <look name="Gray"/>
        <world_transform xyz="{values['OBJ_X']} {values['OBJ_Y']} {values['OBJ_Z']}"
                         rpy="{values['OBJ_ROLL']} {values['OBJ_PITCH']} {values['OBJ_YAW']}"/>
    </static>'''


def _render_target_scene(context, robot_scene):
    """Return a per-transform target scene, leaving source assets untouched."""
    values = {
        'OBJ_X': _finite_float(context, 'obj_x'),
        'OBJ_Y': _finite_float(context, 'obj_y'),
        'OBJ_Z': _finite_float(context, 'obj_z'),
        'OBJ_SCALE': _finite_float(context, 'obj_scale', positive=True),
        # Stonefish XML takes radians; the public launch interface intentionally
        # uses degrees because it is what an operator types into the TUI.
        'OBJ_ROLL': math.radians(_finite_float(context, 'obj_roll')),
        'OBJ_PITCH': math.radians(_finite_float(context, 'obj_pitch')),
        'OBJ_YAW': math.radians(_finite_float(context, 'obj_yaw')),
    }
    mesh = _selected_mesh(context)
    scale = values['OBJ_SCALE']
    values = {key: format(value, '.9g') for key, value in values.items()}
    values['TARGET_BODY'] = _target_body(mesh, values, scale)
    values['ROBOT_SCENE'] = robot_scene

    template_path = Path(SCENARIO_DIR, f'{TARGET_SCENE}.scn.in')
    try:
        template = Template(template_path.read_text(encoding='utf-8'))
    except OSError as exc:
        raise RuntimeError(f'Cannot read target scene template {template_path}') from exc
    rendered = template.substitute(values)

    return _scene_output_path('target', rendered)


def _render_waterlinked_scene(robot_scene):
    """Copy only the include path, retaining the baseline scene's contents."""
    source = Path(SCENARIO_DIR, 'waterlinked.scn')
    try:
        rendered = source.read_text(encoding='utf-8')
    except OSError as exc:
        raise RuntimeError(f'Cannot read baseline scenario {source}') from exc
    include = '<include file="robot/bluerov2_unphy.scn"/>'
    if include not in rendered:
        raise RuntimeError(f'Cannot find robot include in baseline scenario {source}')
    return _scene_output_path('waterlinked', rendered.replace(
        include, f'<include file="{robot_scene}"/>'))


def _launch_stonefish(context):
    scene = LaunchConfiguration('scene').perform(context)
    if scene not in SCENES:
        raise RuntimeError(f'Unknown scene {scene!r}; choose one of {", ".join(SCENES)}')
    robot_pose = _robot_pose(context)
    thrust_boost = LaunchConfiguration('thrust_boost').perform(context) == 'true'
    if scene == DEFAULT_SCENE and robot_pose == DEFAULT_ROBOT_POSE and not thrust_boost:
        scenario = os.path.join(SCENARIO_DIR, 'waterlinked.scn')
    else:
        robot_scene = _render_robot_scene(robot_pose, thrust_boost)
        scenario = (_render_waterlinked_scene(robot_scene)
                    if scene == DEFAULT_SCENE else _render_target_scene(context, robot_scene))
    return [Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[WORLD_DATA, scenario, '300', '1200', '900', 'medium'],
        output='screen',
    )]


def generate_launch_description():
    scene_arg = DeclareLaunchArgument(
        'scene', default_value=DEFAULT_SCENE, choices=list(SCENES),
        description='Stonefish scene: waterlinked (baseline) or target (templated object)',
    )
    obj_x_arg = DeclareLaunchArgument(
        'obj_x', default_value='8.0',
        description='target scene object X position in world_ned (m)',
    )
    obj_mesh_arg = DeclareLaunchArgument(
        'obj_mesh', default_value=BUILTIN_PIPE,
        description='target object: pipe or an .obj/.stl filename from world data/obj',
    )
    obj_y_arg = DeclareLaunchArgument(
        'obj_y', default_value='-2.0',
        description='target scene object Y position in world_ned (m)',
    )
    obj_z_arg = DeclareLaunchArgument(
        'obj_z', default_value='8.0',
        description='target scene object Z position in world_ned (m)',
    )
    obj_scale_arg = DeclareLaunchArgument(
        'obj_scale', default_value='1.0',
        description='target scene object uniform scale (> 0)',
    )
    obj_roll_arg = DeclareLaunchArgument(
        'obj_roll', default_value='0.0',
        description='target scene object roll (degrees)',
    )
    obj_pitch_arg = DeclareLaunchArgument(
        'obj_pitch', default_value='0.0',
        description='target scene object pitch (degrees)',
    )
    obj_yaw_arg = DeclareLaunchArgument(
        'obj_yaw', default_value='0.0',
        description='target scene object yaw (degrees)',
    )
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='0.0',
                                        description='robot X position in world_ned (m)')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='0.0',
                                        description='robot Y position in world_ned (m)')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='8.0',
                                        description='robot Z position in world_ned (m)')
    robot_roll_arg = DeclareLaunchArgument('robot_roll', default_value='0.0',
                                           description='robot roll (degrees)')
    robot_pitch_arg = DeclareLaunchArgument('robot_pitch', default_value='0.0',
                                            description='robot pitch (degrees)')
    robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0',
                                          description='robot yaw (degrees)')
    thrust_boost_arg = DeclareLaunchArgument(
        'thrust_boost', default_value='false', choices=['true', 'false'],
        description=(f'Scale the simulated thruster RPM ceiling by '
                     f'{THRUST_BOOST_MULTIPLIER}x for fast repositioning; '
                     'not a physically realistic setting'),
    )

    return LaunchDescription([
        scene_arg, obj_mesh_arg, obj_x_arg, obj_y_arg, obj_z_arg, obj_scale_arg,
        obj_roll_arg, obj_pitch_arg, obj_yaw_arg,
        robot_x_arg, robot_y_arg, robot_z_arg,
        robot_roll_arg, robot_pitch_arg, robot_yaw_arg,
        thrust_boost_arg,
        OpaqueFunction(function=_launch_stonefish),
    ])
