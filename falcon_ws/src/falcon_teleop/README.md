# falcon_teleop

Keyboard teleop for EUFS simulation with embedded front-view. Game-style controls instead of the dial-based EUFS Robot Steering GUI.

## Requirements

- tkinter (usually bundled with Python)
- cv_bridge: `sudo apt install ros-humble-cv-bridge`
- Pillow (for camera view): `pip install Pillow` or `sudo apt install python3-pil`

## Controls

| Key | Action |
|-----|--------|
| W | Forward (hold) |
| S | Brake (hold) |
| A | Steer left (hold) |
| D | Steer right (hold) |
| R | Increase speed limit |
| F | Decrease speed limit |
| Space | Stop (reset speed & steering) |

**Click the teleop window** to give it focus before driving—keys only work when the window is focused. The camera view is optional; you can drive without it.

## Front view

When running with `launch_group:=default` (real sensors), the ZED left camera feed appears in the teleop window, giving a driving-game style view. With `launch_group:=no_perception`, the camera is disabled and a placeholder is shown.

## Usage

1. Start simulation: `ros2 launch falcon_bringup simulation.launch.py`
2. In another terminal, run the teleop:

```bash
ros2 launch falcon_teleop teleop.launch.py
```

Or run the node directly:

```bash
ros2 run falcon_teleop keyboard_teleop
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| cmd_topic | /cmd | Ackermann drive command topic |
| image_topic | /zed/left/image_rect_color | Camera image for front view |
| publish_rate | 50.0 | Command publish rate (Hz) |
| speed_increment | 1.0 | Speed change per R/F press (m/s) |
| steering_increment | 0.1 | Steering rate when holding A/D (rad) |
| max_speed | 10.0 | Speed limit (m/s) |
| min_speed | 0.0 | Minimum speed |
| max_steering | 0.42 | Max steering angle (rad) |

Override: `ros2 launch falcon_teleop teleop.launch.py max_speed:=15.0`
