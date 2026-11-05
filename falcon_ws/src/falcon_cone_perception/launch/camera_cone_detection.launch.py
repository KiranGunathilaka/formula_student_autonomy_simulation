import os
import subprocess
import sys

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_yolo_device() -> str:
    """Choose YOLO device without hard-coding cuda:0.

    Order:
      1) Non-empty FALCON_YOLO_DEVICE (e.g. cuda:0, cpu, cuda:1).
      2) cuda:0 when PyTorch sees any CUDA device (host GPU or Docker --gpus), asked
         of the launch's own Python or else of the python3 the YOLO node runs on.
      3) cpu otherwise (no GPU in container, driver down, CPU-only torch).

    Why: camera_cone_detection used to pass cuda:0 always; without a working
    NVIDIA stack, YOLO never produced /yolo/detections and the car had no path.
    """
    device_override_from_env = os.environ.get('FALCON_YOLO_DEVICE', '').strip()
    if device_override_from_env:
        return device_override_from_env
    try:
        import torch

        return 'cuda:0' if torch.cuda.is_available() else 'cpu'
    except Exception:
        pass
    # `ros2 launch` runs under the system Python, which may not have torch (in the
    # Docker image only the venv does); the YOLO node runs under the python3 on PATH,
    # so ask that one.
    try:
        out = subprocess.run(
            ['python3', '-c', 'import torch; print(torch.cuda.is_available())'],
            capture_output=True, text=True, timeout=60)
        if out.stdout.strip().endswith('True'):
            return 'cuda:0'
    except Exception:
        pass
    return 'cpu'


def generate_launch_description():
    resolved_yolo_device = _resolve_yolo_device()
    print(
        '[camera_cone_detection] YOLO device: '
        f'{resolved_yolo_device!r} '
        '(override with env FALCON_YOLO_DEVICE)',
        file=sys.stderr,
        flush=True,
    )

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
                "device": resolved_yolo_device,
                "threshold": "0.5",
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