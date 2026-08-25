from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'avoidance_route'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True, maintainer='qor',
    maintainer_email='qor@example.com',
    description='Actual-odometry route recording, replay, and terminal teleoperation.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'route_recorder = avoidance_route.route_recorder:main',
        'route_visualizer = avoidance_route.route_visualizer:main',
        'route_follower = avoidance_route.route_follower:main',
    ]},
)
