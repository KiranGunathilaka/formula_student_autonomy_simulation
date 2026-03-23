from setuptools import setup
import os
from glob import glob

package_name = 'falcon_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    scripts=['scripts/enable_manual_drive'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Launch files for Falcon autonomy stack',
    license='MIT',
    extras_require={'test': ['pytest']},
)
