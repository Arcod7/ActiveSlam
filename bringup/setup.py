from setuptools import setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo.launch.py']),
        ('share/' + package_name + '/rviz', [
            'rviz/demo.rviz', 'rviz/demo_tsdf.rviz', 'rviz/demo_slam.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='antoine.esman7@gmail.com',
    description='Unified demo bring-up: Stonefish + mapper (OctoMap/TSDF) + operator mode (teleop/frontier)',
    license='MIT',
)
