"""
Depth image → PointCloud2 (includes tf.launch.py: Stonefish + TF chain).

  /sensor_msgs/image_depth  ──► depth_image_proc ──► /cloud_in
  /sensor_msgs/camera_info  ──►┘

NaN replacement (REP 118) is handled inside stonefish_ros2 at publish time.

When sonar_noise:=true (set by demo.launch.py under slam:=slam), a datasheet-
grounded noise model is inserted between depth_image_proc and every consumer,
standing in for a real WaterLinked Sonar 3D-15 (see slam_backend's
sonar_noise.py):

  depth_image_proc ──► /cloud_in_raw ──► sonar_noise ──► /cloud_in

sonar_noise:=false (default, and always under slam:=none) keeps the original
direct wiring so the topology and topic count are unchanged.

Test:
  ros2 topic echo /cloud_in --no-arr
  Expected: header.frame_id = "bluerov2/Dcam", width=257, height=67
  No sphere should appear at the bluerov2/Dcam TF origin in RViz.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sonar_noise_arg = DeclareLaunchArgument(
        'sonar_noise', default_value='false',
        description='Insert the sonar noise model between depth_image_proc and /cloud_in',
    )
    noise_profile_arg = DeclareLaunchArgument(
        'noise_profile', default_value='realistic',
        description='Noise profile for sonar_noise (ideal, sonar_only, odom_pos_only, odom_only, realistic, degraded)',
    )
    noise_seed_arg = DeclareLaunchArgument(
        'noise_seed', default_value='-1',
        description='Override the noise profile seed (-1 = use the profile default)',
    )

    tf_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stonefish_groundtruth_mapping'),
                'launch', 'tf.launch.py'
            )
        )
    )

    # depth_image_proc's output topic: straight to /cloud_in normally, or to
    # /cloud_in_raw (feeding sonar_noise) when the noise model is active.
    depth_proc_output = PythonExpression([
        "'/cloud_in_raw' if '", LaunchConfiguration('sonar_noise'), "' == 'true' else '/cloud_in'",
    ])

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
                    ('points',     depth_proc_output),
                ],
            ),
        ],
        output='screen',
    )

    noise_file = PathJoinSubstitution([
        FindPackageShare('slam_backend'), 'config',
        ['noise_', LaunchConfiguration('noise_profile'), '.yaml'],
    ])

    sonar_noise_node = Node(
        package='slam_backend',
        executable='sonar_noise',
        name='sonar_noise',
        output='screen',
        parameters=[{
            'noise_profile_path': noise_file,
            'noise_seed': LaunchConfiguration('noise_seed'),
            'input_topic': '/cloud_in_raw',
            'output_topic': '/cloud_in',
        }],
        condition=IfCondition(LaunchConfiguration('sonar_noise')),
    )

    return LaunchDescription([
        sonar_noise_arg, noise_profile_arg, noise_seed_arg,
        tf_chain, depth_to_cloud, sonar_noise_node,
    ])
