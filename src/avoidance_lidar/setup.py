from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'avoidance_lidar'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'], zip_safe=True, maintainer='qor',
    maintainer_email='qor@example.com',
    description='Front LaserScan ROI and obstacle visualization diagnostics.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'front_lidar_detector = avoidance_lidar.front_lidar_node:main',
        'lidar_safety = avoidance_lidar.safety_node:main',
    ]},
)
