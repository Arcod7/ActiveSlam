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
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_WORLD_DIR = os.environ.get('STONEFISH_WORLD_DIR', get_package_share_directory('world'))
WORLD_DATA = os.path.join(_WORLD_DIR, 'data')
SCENARIO   = os.path.join(_WORLD_DIR, 'scenario', 'waterlinked.scn')


def generate_launch_description():
    stonefish = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[WORLD_DATA, SCENARIO, '300', '1200', '900', 'medium'],
        output='screen',
    )

    return LaunchDescription([stonefish])
