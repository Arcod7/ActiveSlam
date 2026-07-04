from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'slam_backend'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='ave4000@hw.ac.uk',
    description='GTSAM iSAM2 pose-graph SLAM backend with simulated sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pressure_sim = slam_backend.sensor_models.pressure_sim:main',
            'imu_sim      = slam_backend.sensor_models.imu_sim:main',
            'dvl_sim      = slam_backend.sensor_models.dvl_sim:main',
            'pose_graph   = slam_backend.pose_graph:main',
        ],
    },
)
