from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'falcon_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Keyboard teleop with embedded front-view for EUFS simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'keyboard_teleop = falcon_teleop.keyboard_teleop:main',
        ],
    },
)
