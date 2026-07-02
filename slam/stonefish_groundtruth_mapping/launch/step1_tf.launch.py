"""
Step 1 — TF chain.

Starts:
  - Stonefish simulator  (bluerov2 + off_shore_station)
  - odom_to_tf           : /StoneFish/Odometry → TF(world_ned → bluerov2/base_link)
  - static camera TF     : bluerov2/base_link  → bluerov2/Dcam

World path is read from the STONEFISH_WORLD_DIR environment variable.
Default: <repo root>/sim/world, resolved relative to this launch file (works
with `colcon build --symlink-install`, this project's documented build
command, regardless of where the repo is cloned).
Override before launching:
  export STONEFISH_WORLD_DIR=/path/to/world

Verify with:
  ros2 run tf2_tools view_frames
  Expected tree: world_ned → bluerov2/base_link → bluerov2/Dcam
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

# realpath() follows the symlink-install link back to the source file, so
# this resolves correctly even though colcon runs launch files from install/.
_THIS_DIR = os.path.dirname(os.path.realpath(__file__))
_DEFAULT_WORLD_DIR = os.path.normpath(
    os.path.join(_THIS_DIR, '..', '..', '..', 'sim', 'world')
)
_WORLD_DIR  = os.environ.get('STONEFISH_WORLD_DIR', _DEFAULT_WORLD_DIR)
WORLD_DATA = os.path.join(_WORLD_DIR, 'data')
SCENARIO   = os.path.join(_WORLD_DIR, 'scnenario', 'waterlinked.scn')


def generate_launch_description():
    stonefish = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[WORLD_DATA, SCENARIO, '300', '1200', '900', 'medium'],
        output='screen',
    )

    odom_to_tf = Node(
        package='stonefish_groundtruth_mapping',
        executable='odom_tf_sync',
        name='odom_tf_sync',
        output='screen',
    )

    # Camera mount from bluerov2_unphy.scn:
    # <origin rpy="1.571 0.0 1.571" xyz="0.2 0.0 0.3" /> on base_link
    static_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf',
        arguments=[
            '--x', '0.2', '--y', '0.0', '--z', '0.3',
            '--roll', '1.571', '--pitch', '0.0', '--yaw', '1.571',
            '--frame-id', 'bluerov2/base_link',
            '--child-frame-id', 'bluerov2/Dcam',
        ],
    )

    return LaunchDescription([stonefish, odom_to_tf, static_camera_tf])
