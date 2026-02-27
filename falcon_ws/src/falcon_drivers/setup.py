from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'falcon_drivers'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='ZED and LiDAR driver configs and wrappers',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={},
)
