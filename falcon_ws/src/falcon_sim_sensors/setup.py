from setuptools import setup

package_name = 'falcon_sim_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    description='Simulated ADS-DV CAN wheel speeds and steer angle, and an Xsens MTi-8 IMU',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_sensors_sim = falcon_sim_sensors.vehicle_sensors_node:main',
        ],
    },
)
