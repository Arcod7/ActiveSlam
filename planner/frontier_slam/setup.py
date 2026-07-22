from setuptools import find_packages, setup

package_name = 'frontier_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frontier_slam.launch.py',
                                               'launch/wall_follow.launch.py',
                                               'launch/ardusub_adapter.launch.py']),
        ('share/' + package_name + '/config', ['config/ardusub.yaml']),
    ],
    # pymavlink is an optional hardware-only dependency in the repo-level
    # requirements.txt. Keeping it out of package requirements lets simulation
    # nodes run on machines that never connect to ArduSub.
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='antoine.esman7@gmail.com',
    description='Frontier-based exploration: OctoMap projected map → waypoint → thruster control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_extractor  = frontier_slam.frontier_extractor:main',
            'waypoint_controller = frontier_slam.waypoint_controller:main',
            'wall_oriented_controller = frontier_slam.wall_oriented_controller:main',
            'tsdf_mapper         = frontier_slam.tsdf_mapper:main',
            'wall_follower       = frontier_slam.wall_follower:main',
            'motion_safety_gate = frontier_slam.safety_gate:main',
            'heavy_sim_mixer    = frontier_slam.heavy_sim_mixer:main',
            'ardusub_adapter    = frontier_slam.ardusub_adapter:main',
            'revisit_planner     = frontier_slam.revisit_planner:main',
            'drift_return_scenario = frontier_slam.drift_return_scenario:main',
        ],
    },
)
