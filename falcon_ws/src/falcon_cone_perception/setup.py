from setuptools import setup, find_packages

setup(
    name='falcon_cone_perception',
    version='0.1.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/falcon_cone_perception']),
        ('share/falcon_cone_perception', ['package.xml']),
        ('share/falcon_cone_perception/launch', [
            'launch/cone_detection.launch.py',
        ]),
        ('share/falcon_cone_perception/config', [
            'config/yolo_detector.yaml',
            'config/topic_remaps.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
    ],
    author='FalconE Racing',
    author_email='falcone@gmail.com',
    maintainer='FalconE Racing',
    maintainer_email='falcone@gmail.com',
    license='MIT',
    description='Cone perception from camera and pointcloud',
    long_description='Cone detection module for autonomous vehicle perception using YOLOv8',
    entry_points={
        'console_scripts': [
            'cone_detection_node=falcon_cone_perception.cone_detection_node:main',
        ],
    },
)
