# falcon_autonomy

Formula Student Driverless stack: ZED1 + 2D LiDAR, cone-based mapping.

**ROS 2 Humble.**

## Build & Setup

From the repository root (source it, don't run):

```bash
source scripts/setup_falcon.sh           # Incremental build + setup (picks up new packages)
source scripts/setup_falcon.sh --clean   # Full clean rebuild
source scripts/setup_falcon.sh --setup   # Setup only, no build (new terminal)
```

Always use `setup_falcon.sh` (not only `install/setup.bash`)—it sets `EUFS_MASTER` and creates an `install/eufs_plugins` symlink required for Gazebo plugin loading with merge-install.

## Run Simulation (EUFS Gazebo)

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup simulation.launch.py
```

Defaults: small track, ads-dv robot, velocity control, dry track, ground truth TFs, Gazebo GUI, RViz, RQT steering. Override with `gazebo_gui:=false` or `rviz:=false` if needed.

### Simulated vs real perception

| Command | Cones | Lidar | Cameras |
|---------|-------|-------|---------|
| `ros2 launch falcon_bringup simulation.launch.py` | `/cones` (simulated) | Off | Off |
| `ros2 launch falcon_bringup simulation.launch.py launch_group:=default` | None from plugin | `/gazebo_scan` | Images on |

Use `launch_group:=no_perception` (default) for simulated cones; use `launch_group:=default` for real lidar and cameras.

### If gzserver exits with code 255

1. **Use `setup_falcon.sh`** so `EUFS_MASTER` and `install/eufs_plugins` are correct:
   ```bash
   source scripts/setup_falcon.sh   # from repo root, not only install/setup.bash
   ```

2. **Permission issues**: If Gazebo or build was run with `sudo`, fix ownership:
   ```bash
   sudo chown -R $USER:$USER ~/.gazebo ~/.ros
   sudo chown -R $USER:$USER falcon_ws/install   # if install is root-owned
   ```

3. **GPU/graphics**: Try headless first:
   ```bash
   ros2 launch falcon_bringup simulation.launch.py gazebo_gui:=false
   ```
   If that works, the issue is likely GUI/GPU-related.

4. **Capture the real error**: Run gzserver manually to see the crash reason:
   ```bash
   source scripts/setup_falcon.sh
   export GAZEBO_MODEL_PATH="$EUFS_MASTER/install/share/eufs_tracks/models"
   export GAZEBO_RESOURCE_PATH="$EUFS_MASTER/install/share/eufs_tracks/meshes:$EUFS_MASTER/install/share/eufs_tracks/materials:$EUFS_MASTER/install/share/eufs_racecar/meshes:$EUFS_MASTER/install/share/eufs_racecar/materials:/usr/share/gazebo-11"
   export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:$EUFS_MASTER/install/eufs_plugins"
   gzserver --verbose "$EUFS_MASTER/install/share/eufs_tracks/worlds/small_track.world" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so
   ```

### Keyboard teleop

Instead of the EUFS dial-based steering GUI, use game-style keyboard teleop with front camera view:

```bash
# In a separate terminal, after simulation is running:
ros2 launch falcon_teleop teleop.launch.py
```

Controls: **W** forward, **S** brake, **A/D** steer, **R/F** speed +/-, **Space** stop. The teleop automatically enables Manual Drive; no RQT click needed. Requires `launch_group:=default` for camera view. See `falcon_teleop/README.md`.

## Simulated Perception

The EUFS simulation includes a dummy cone publisher that mimics a typical perception stack (lidar + camera fusion with noise and color misclassification).

**Topic:** `/cones`  
**Message type:** `eufs_msgs/msg/ConeArrayWithCovariance`

To receive simulated cone detections, subscribe to `/cones`:

```bash
ros2 topic echo /cones
```


**Message layout:** Cones are grouped by color: `blue_cones`, `yellow_cones`, `orange_cones`, `big_orange_cones`, `unknown_color_cones`. Each cone has a `geometry_msgs/Point` and `float64[4]` covariance. The header `frame_id` is `base_footprint`.


```bash
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

ConeWithCovariance[] blue_cones
	geometry_msgs/Point point
		float64 x
		float64 y
		float64 z
	float64[4] covariance
ConeWithCovariance[] yellow_cones
	geometry_msgs/Point point
		float64 x
		float64 y
		float64 z
	float64[4] covariance
ConeWithCovariance[] orange_cones
	geometry_msgs/Point point
		float64 x
		float64 y
		float64 z
	float64[4] covariance
ConeWithCovariance[] big_orange_cones
	geometry_msgs/Point point
		float64 x
		float64 y
		float64 z
	float64[4] covariance
ConeWithCovariance[] unknown_color_cones
	geometry_msgs/Point point
		float64 x
		float64 y
		float64 z
	float64[4] covariance
```

**Enable/disable:** Simulated perception is on when launching with `launch_group:=no_perception` (default in `simulation.launch.py`). That runs without real lidar/camera and publishes cones to `/cones`.

### Simulated perception behaviour

| Question | Answer |
|----------|--------|
| All cones or only in-FOV? | **Only cones in sensor FOV** – not the full track |
| Vehicle has lidar? | ads-dv URDF includes RPLidar A1, but when simulated perception is on it is **disabled** – no `/gazebo_scan` |
| Front-only or 360°? | **Front-only** – lidar 180°, camera 120°; no rear cones |

The plugin simulates lidar + camera fusion: cones seen by lidar but not camera → `unknown_color_cones`; cones in camera FOV → known color. FOV is a semicircle in front (x forward, y left).

### Sensor layout (ads-dv)

When simulated perception is **off** (real sensors), the lidar publishes to `/gazebo_scan`. With simulated perception **on**, the lidar and camera plugins are disabled (to save compute); cone data comes from `/cones` instead. Any `/scan` you see may be from another node (e.g. RViz display config) or a different world model.

**Mount positions** (xyz in metres, chassis frame; x=forward, y=left, z=up):

| Sensor | Position (x, y, z) | Notes |
|--------|--------------------|-------|
| RPLidar A1 | 0.443, 0.0, 0.3 | Front, on chassis centreline |
| ZED stereo camera | 0.334, 0.0, 0.295 | Slightly behind lidar |
| IMU | 0.0, 0.0, 0.0 | Chassis origin |
| GPS | 0.463, 0.068, 0.224 | Right of centre |

## Vehicle pose (simulation)

EUFS publishes vehicle pose, so you can run planning/control without a localization node in sim.

| Topic | Type | Description |
|-------|------|--------------|
| `/ground_truth/state` | `eufs_msgs/CarState` | **Perfect pose** (position, orientation, velocities, no noise) |
| `/ground_truth/odom` | `nav_msgs/Odometry` | Noisy odometry (simulates real odom) |
| TF `map` → `base_footprint` | — | Pose transform (uses noisy state when `publish_gt_tf:=true`) |

For a known pose without localization, subscribe to `/ground_truth/state`. The TF and odom are deliberately noisy for more realistic behaviour.

## Packages

- `falcon_msgs` – Cone and ConeArray message definitions
- `falcon_bringup` – Launch files
- `falcon_common` – Shared C++ utilities
- `falcon_teleop` – Keyboard teleop with embedded front-view
- `falcon_drivers` – ZED/LiDAR config placeholders
- `falcon_cone_perception` – Raw cone detection from pointcloud
- `falcon_cone_fusion` – Local cone fusion
- `falcon_cone_map_builder` – Global cone map
- `falcon_localization` – Localization 
- `falcon_planning` – Planning
- `falcon_vehicle_comm` – Vehicle interface 
