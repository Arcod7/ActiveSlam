"""
Step 1 — TF chain verification.

Starts:
  - Stonefish simulator (bluerov2 + off_shore_station)
  - odom_to_tf     : /StoneFish/Odometry → TF(world_ned → bluerov2/base_link)
  - static TF      : bluerov2/base_link  → Dcam  (camera mount from .scn)

Test:
  ros2 run tf2_tools view_frames
  Expected tree: world_ned → bluerov2/base_link → Dcam
"""
from launch import LaunchDescription
from launch_ros.actions import Node

WORLD_DATA = '/home/antoine/ros2_ws/src/world/data'
SCENARIO   = '/home/antoine/ros2_ws/src/world/scnenario/waterlinked.scn'


def generate_launch_description():
    stonefish = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        name='stonefish_simulator',
        arguments=[WORLD_DATA, SCENARIO, '300', '1200', '900', 'medium'],
        output='screen',
    )

    odom_to_tf = Node(
        package='basic_slam',
        executable='odom_to_tf',
        name='odom_to_tf',
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
            '--child-frame-id', 'Dcam',
        ],
    )

    return LaunchDescription([stonefish, odom_to_tf, static_camera_tf])
