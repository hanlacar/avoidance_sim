from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'avoidance_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'models', 'vehicle_box'), glob('models/vehicle_box/*')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='qor',
    maintainer_email='qor@example.com',
    description='Straight single-lane Gazebo obstacle course for turtle_car.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
