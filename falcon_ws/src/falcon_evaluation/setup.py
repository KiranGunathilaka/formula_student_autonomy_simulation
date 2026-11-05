import os
from glob import glob

from setuptools import setup

package_name = 'falcon_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Cone coverage metric and lidar sweeps for the Falcon simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_coverage = falcon_evaluation.cone_coverage_node:main',
            'lidar_sweep = falcon_evaluation.lidar_sweep:main',
        ],
    },
)
