import os
from glob import glob

from setuptools import setup

package_name = 'falcon_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'config', 'vehicles'), glob('config/vehicles/*.yaml')),
        (os.path.join('share', package_name, 'config', 'lidars'),
         glob('config/lidars/*.yaml') + glob('config/lidars/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Vehicle and lidar profiles and the URDF builder for the Falcon car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'falcon_urdf = falcon_description.cli:main',
        ],
    },
)
