"""
Step 2 — Depth image → PointCloud2.

Adds depth_image_proc::PointCloudXyzNode on top of Step 1.

  /sensor_msgs/image_depth  ──┐
                               ├─ depth_image_proc ──► /cloud_in  (frame: bluerov2/Dcam)
  /sensor_msgs/camera_info  ──┘

Test:
  ros2 topic echo /cloud_in --no-arr
  Expected: header.frame_id = "bluerov2/Dcam", width=256, height=64
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
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
                    ('image_rect',   '/sensor_msgs/image_depth'),
                    ('camera_info',  '/sensor_msgs/camera_info'),
                    ('points',       '/cloud_in'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([step1, depth_to_cloud])
