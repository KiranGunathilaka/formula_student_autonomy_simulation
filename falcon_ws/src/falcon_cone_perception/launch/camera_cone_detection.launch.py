from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    falcon_pkg = FindPackageShare("falcon_cone_perception")
    yolo_bringup_pkg = FindPackageShare("yolo_bringup")

    model_path = PathJoinSubstitution([
        falcon_pkg,
        "models",
        "fsoco_yolo11n_best.pt"
    ])

    yolo_launch = PathJoinSubstitution([
        yolo_bringup_pkg,
        "launch",
        "yolov11.launch.py"
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yolo_launch),
            launch_arguments={
                "model": model_path,
                "input_image_topic": "/zed/image_raw",
                "device": "cuda:0",
                "threshold": "0.25",
                "iou": "0.45",
                "imgsz_height": "640",
                "imgsz_width": "640",
                "use_tracking": "False",
                "use_3d": "False",
                "use_debug": "True",
                "namespace": "yolo",
            }.items(),
        )
    ])