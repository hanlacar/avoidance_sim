from glob import glob
import os
from setuptools import find_packages, setup


package_name = 'avoidance_planner'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qor',
    maintainer_email='qor@example.com',
    description='Separated LiDAR tracking, collision evaluation, and stopped local planning.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': [
        'avoidance_coordinator = avoidance_planner.coordinator_node:main',
    ]},
)

