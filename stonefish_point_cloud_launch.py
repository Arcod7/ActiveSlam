from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='stonefish_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='depth_to_pointcloud_node',
                    # Remappage vers vos topics Stonefish réels
                    remappings=[
                        ('image_rect', '/sensor_msgs/image_depth'),
                        ('camera_info', '/sensor_msgs/camera_info'),
                        ('points', '/sensor_msgs/pointcloud') 
                    ]
                ),
            ],
            output='screen',
        )
    ])
