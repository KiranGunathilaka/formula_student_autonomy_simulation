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

**Click the teleop window** to give it focus before driving.

## Front view

A dedicated **teleop camera** (invisible, no mesh) is always active in the ads-dv robot. It publishes to `/teleop_camera/image_raw` for **both** `launch_group:=default` and `launch_group:=no_perception`, so you get a front-view feed in the teleop window regardless of launch mode.

The simulation must be running (Gazebo + spawned robot) for the camera to publish.

### Camera not showing?

1. **Start simulation first**, then teleop: `ros2 launch falcon_teleop teleop.launch.py`
2. **Verify topic**: `ros2 topic list | grep image` and `ros2 topic hz /teleop_camera/image_raw`
3. **Dependencies**: Ensure `cv_bridge`, `opencv-python`, and `Pillow` are installed (camera is required for teleop)

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
| image_topic | /teleop_camera/image_raw | Camera image (always-on teleop camera) |
| publish_rate | 50.0 | Command publish rate (Hz) |
| speed_increment | 1.0 | Speed change per R/F press (m/s) |
| steering_increment | 0.1 | Steering rate when holding A/D (rad) |
| max_speed | 10.0 | Speed limit (m/s) |
| min_speed | 0.0 | Minimum speed (m/s) |
| max_steering | 0.42 | Max steering angle (rad) |

Override: `ros2 launch falcon_teleop teleop.launch.py max_speed:=15.0`
