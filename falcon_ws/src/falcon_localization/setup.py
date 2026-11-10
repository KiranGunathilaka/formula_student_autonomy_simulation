import os
from glob import glob

from setuptools import setup

package_name = 'falcon_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Odometry EKF (IMU, wheel speeds, steer angle) and cone EKF-SLAM for Falcon',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odometry_ekf = falcon_localization.odometry_node:main',
            'cone_slam = falcon_localization.slam_node:main',
        ],
    },
)
