Launch files for the Falcon autonomy stack.

| Launch file | Description |
|-------------|-------------|
| `simulation.launch.py` | EUFS Gazebo sim only (small track, ads-dv, configurable sensors) |
| `autonomy.launch.py` | Full autonomous stack using ground-truth cones from EUFS (`/cones`) |
| `autonomy_perception.launch.py` | Full autonomous stack using real perception (YOLO + LiDAR + fusion) |

## autonomy.launch.py

Runs the car autonomously using ground-truth cone positions from the simulator.
No camera or LiDAR detection — uses the EUFS pseudo-perception plugin.

```bash
ros2 launch falcon_bringup autonomy.launch.py
ros2 launch falcon_bringup autonomy.launch.py total_laps:=3
```

Node graph:
```
EUFS sim → /cones
  ├─ cone_bridge → cone_fusion → /perception/cones_fused ──→ path_planner
  └─ cone_map_builder → /map/cone_map ────────────────────→    │
                                                          /planning/path
                                                               │
                                                          pure_pursuit → /cmd
```

## autonomy_perception.launch.py

Runs the car autonomously using real perception from cameras and LiDAR.
The EUFS simulator provides raw sensor data; the Falcon perception stack
(YOLO cone detection, depth-based localization, LiDAR clustering, and
sensor fusion) processes it into cone observations.

```bash
ros2 launch falcon_bringup autonomy_perception.launch.py
ros2 launch falcon_bringup autonomy_perception.launch.py total_laps:=5
ros2 launch falcon_bringup autonomy_perception.launch.py total_laps:=3 gazebo_gui:=false
```

| Argument | Default | Description |
|----------|---------|-------------|
| `total_laps` | `0` | Number of laps (0 = unlimited) |
| `gazebo_gui` | `true` | Show Gazebo window |
| `rviz` | `true` | Launch RViz |
| `launch_group` | `default` | EUFS sensor group (`default` = cameras + LiDAR) |

Node graph:
```
EUFS sim (cameras + LiDAR)
  ├─ YOLO → depth_localizer → /falcon/camera_cones ──┐
  ├─ lidar_detector → /falcon/lidar_cones ────────────┤
  │                                              cone_fuser
  │                                    ┌───────────┤
  │                        /falcon/fused_cones   /perception/cones_fused
  │                              │                     │
  │                     cone_map_builder          path_planner
  │                              │                     │
  │                     /map/cone_map ─────────→       │
  │                                              /planning/path
  │                                                    │
  └─────────────── /cmd ◄──────────────────── pure_pursuit
```

See the top-level README for the full architecture diagram, RViz setup, and tuning guide.
