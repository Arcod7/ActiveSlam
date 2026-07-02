"""
Depth image → PointCloud2 (includes tf.launch.py: Stonefish + TF chain).

  /sensor_msgs/image_depth  ──► depth_image_proc ──► /cloud_in
  /sensor_msgs/camera_info  ──►┘

NaN replacement (REP 118) is handled inside stonefish_ros2 at publish time.

Test:
  ros2 topic echo /cloud_in --no-arr
  Expected: header.frame_id = "bluerov2/Dcam", width=256, height=64
  No sphere should appear at the bluerov2/Dcam TF origin in RViz.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    tf_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stonefish_groundtruth_mapping'),
                'launch', 'tf.launch.py'
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
                    ('image_rect', '/sensor_msgs/image_depth'),
                    ('points',     '/cloud_in'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([tf_chain, depth_to_cloud])
