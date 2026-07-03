from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'eval_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoine',
    maintainer_email='ave4000@hw.ac.uk',
    description='Benchmark node + TUM writer + offline plotting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchmark    = eval_tools.benchmark:main',
            'plot_results = eval_tools.plot_results:main',
        ],
    },
)
