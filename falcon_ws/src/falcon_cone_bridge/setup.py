from setuptools import setup
import os
from glob import glob

package_name = 'falcon_cone_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Converts EUFS simulator cone messages to Falcon ConeArray messages',
    license='MIT',
    scripts=['scripts/cone_bridge_node'],
)
