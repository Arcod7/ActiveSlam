"""
Step 2 — Depth image → PointCloud2.

  /sensor_msgs/image_depth  ──► depth_fix ──► /sensor_msgs/image_depth_fixed
                                                          │
  /sensor_msgs/camera_info  ──────────────────────────────┤
                                                          ▼
                                               depth_image_proc ──► /cloud_in

depth_fix replaces 0-valued pixels (no Stonefish depth return) with NaN so
depth_image_proc never produces the (0,0,0) point that octomap_server would
mark as an occupied voxel at the camera origin every frame.

Test:
  ros2 topic echo /cloud_in --no-arr
  Expected: header.frame_id = "bluerov2/Dcam", width=256, height=64
  No sphere should appear at the bluerov2/Dcam TF origin in RViz.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    step1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('basic_slam'),
                'launch', 'step1_tf.launch.py'
            )
        )
    )

    depth_fix = Node(
        package='basic_slam',
        executable='depth_fix',
        name='depth_fix',
        output='screen',
    )

    depth_to_cloud = ComposableNodeContainer(
        name='depth_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='depth_to_cloud',
                remappings=[
                    ('image_rect', '/depth_cam/image_rect'),
                    ('points',     '/cloud_in'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([step1, depth_fix, depth_to_cloud])
