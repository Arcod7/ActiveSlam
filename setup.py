from setuptools import find_packages, setup

package_name = 'basic_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', '_bin']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/step1_tf.launch.py',
            'launch/step2_pointcloud.launch.py',
            'launch/step3_octomap.launch.py',
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
            'odom_to_tf       = basic_slam.odom_to_tf:main',
            'odom_to_tf_noisy = basic_slam.odom_to_tf_noisy:main',
            'odom_tf_sync     = basic_slam.odom_tf_sync:main',
            'timestamp_debug  = basic_slam.timestamp_debug:main',
        ],
    },
)
