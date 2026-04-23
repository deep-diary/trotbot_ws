from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'trotbot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['trotbot']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Include config files
        ('share/' + package_name + '/config', glob('config/**/*', recursive=True)),
        # Include URDF files
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        # Include mesh files
        ('share/' + package_name + '/meshes', glob('meshes/**/*', recursive=True)),
        # Include RViz files
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        # Include world files
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TrotBot Team',
    maintainer_email='your-email@domain.com',
    description='TrotBot Quadruped Robot Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
