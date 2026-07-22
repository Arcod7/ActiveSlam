"""
Depth image -> PointCloud2 without the TF chain or the simulator.

  /sensor_msgs/image_depth  --> depth_image_proc --> /cloud_in_raw
  /sensor_msgs/camera_info  -->                            |
                                                      sonar_noise
                                                           |
                                                       /cloud_in

NaN replacement (REP 118) is handled inside stonefish_ros2 at publish time.

Split out of pointcloud.launch.py so the noise model can be swapped (a
noise_profile change, or slam:=slam toggling sonar_noise on) without
restarting the simulator. pointcloud.launch.py = tf.launch.py + this file.

The topology is the same either way; sonar_noise:=true (set by demo.launch.py
under slam:=slam) decides only whether the relay applies a datasheet-grounded
model standing in for a real WaterLinked Sonar 3D-15, or passes the cloud
through untouched (see slam_backend's sonar_noise.py).

It relays even with the model off because it is also what publishes /cloud_in
on a QoS the consumers can match. depth_image_proc offers its cloud as
SensorDataQoS — best effort — under Humble, while octomap_server, tsdf_mapper
and pose_graph all subscribe reliably; wired directly, nothing connects and no
map is ever built. Under Jazzy the same publisher is reliable. Owning the
republisher keeps that out of the distribution's hands.

Test:
  ros2 topic echo /cloud_in --no-arr
  Expected: header.frame_id = "bluerov2/Dcam", width=257, height=67
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
    near_cutoff_arg = DeclareLaunchArgument(
        'near_cutoff', default_value='-1.0',
        description='Drop noised returns nearer than this (m), over any profile, to '
                    'clear the near-field reverberation spray around the vehicle. '
                    '-1 = use the profile default (0 = keep all near returns).',
    )

    # depth_image_proc always publishes /cloud_in_raw and sonar_noise always
    # republishes it as /cloud_in, whether or not it adds noise. Publishing
    # /cloud_in directly is not an option: depth_image_proc offers it as
    # SensorDataQoS (best effort) under Humble, and every consumer —
    # octomap_server, tsdf_mapper, pose_graph — subscribes reliably, so nothing
    # matches and no map is ever built. Relaying through a node whose QoS this
    # project controls removes the dependency on a distribution's default.
    passthrough = ParameterValue(
        PythonExpression(
            ["'true' if '", LaunchConfiguration('sonar_noise'), "' != 'true' else 'false'"]),
        value_type=bool)

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
                    # Stated rather than left to image_transport to derive from
                    # image_rect: the derivation runs on the unresolved name
                    # under Humble, which lands on /camera_info and never sees
                    # this remap. Without the info there is no camera model and
                    # the node publishes no cloud at all.
                    ('camera_info', '/sensor_msgs/camera_info'),
                    ('points',     '/cloud_in_raw'),
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
            'min_range_m': ParameterValue(LaunchConfiguration('near_cutoff'), value_type=float),
            'input_topic': '/cloud_in_raw',
            'output_topic': '/cloud_in',
            'passthrough': passthrough,
        }],
    )

    # RViz-facing range images: the noised cloud is only a PointCloud2, so render
    # it (and the clean reference) back to depth-camera-style Image topics.
    range_image_slam = Node(
        package='slam_backend', executable='range_image', name='range_image_slam',
        parameters=[{'input_topic': '/cloud_in', 'output_topic': '/cloud_in/range_image'}],
    )
    range_image_raw = Node(
        package='slam_backend', executable='range_image', name='range_image_raw',
        parameters=[{'input_topic': '/cloud_in_raw', 'output_topic': '/cloud_in_raw/range_image'}],
    )

    return LaunchDescription([
        sonar_noise_arg, noise_profile_arg, noise_seed_arg, near_cutoff_arg,
        depth_to_cloud, sonar_noise_node, range_image_slam, range_image_raw,
    ])
