from setuptools import find_packages, setup

package_name = 'stonefish_groundtruth_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', '_bin']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/tf.launch.py',
            'launch/pointcloud.launch.py',
            'launch/octomap.launch.py',
            'launch/tsdf.launch.py',
            'launch/gt_map.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='antoine.esman7@gmail.com',
    description='Ground-truth SLAM pipeline: Stonefish → depth_image_proc → OctoMap',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_to_tf       = stonefish_groundtruth_mapping.odom_to_tf:main',
            'odom_to_tf_noisy = stonefish_groundtruth_mapping.odom_to_tf_noisy:main',
            'odom_tf_sync     = stonefish_groundtruth_mapping.odom_tf_sync:main',
            'cloud_relabel    = stonefish_groundtruth_mapping.cloud_relabel:main',
            'timestamp_debug  = stonefish_groundtruth_mapping.timestamp_debug:main',
        ],
    },
)
