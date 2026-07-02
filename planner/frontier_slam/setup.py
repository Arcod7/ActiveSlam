from setuptools import find_packages, setup

package_name = 'frontier_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/frontier_slam.launch.py']),
    ],
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
            'tsdf_mapper         = frontier_slam.tsdf_mapper:main',
        ],
    },
)
