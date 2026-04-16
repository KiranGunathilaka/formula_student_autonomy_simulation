# falcon_autonomy

Formula Student Driverless stack: ZED1 + 2D LiDAR, cone-based mapping.

**ROS 2 Humble.**

## Quick Start (Docker - Recommended)

The easiest way to build and run the autonomy stack without affecting your local environment is via Docker. The containerized setup will work on Ubuntu 22.04 and higher (not tested on Ubuntu 25.04 though).

**1. Build the Docker container:**
```bash
./docker/docker_build.sh
```

**2. Run the Docker container:**
```bash
./docker/docker_run.sh
```

**3. Run the autonomy script (inside the container):**
```bash
ros2 launch falcon_bringup autonomy.launch.py
```
This will launch the Gazebo simulation and the complete autonomous stack (perception, mapping, planning, control).

## Local Build & Setup

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

Controls: **W** forward, **S** brake, **A/D** steer, **R/F** speed +/-, **Space** stop. Teleop automatically enables Manual Drive (retries until sim is ready). See `falcon_teleop/README.md`.

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

## Autonomous Driving (Simulation)

There are two autonomy launch files — one uses ground-truth cones from the simulator, the other uses the real perception stack (YOLO + LiDAR).

### Option A: Ground-truth dev mode (no perception)

Uses EUFS simulated `/cones` (pseudo-perception) with the planning launch file directly:

## Map Builder

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup simulation.launch.py launch_group:=no_perception
ros2 launch falcon_cone_map_builder cone_map_builder.launch.py use_ground_truth:=true
ros2 launch falcon_teleop teleop.launch.py
```

## Path Planner

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup simulation.launch.py launch_group:=no_perception
ros2 launch falcon_planning planning.launch.py use_ground_truth:=true totla_laps:=3
```
Default lap count is 0 and parsed as infinite

### Option B: Full Autonomy with Real Perception (Recommended)

Uses YOLO camera detection + LiDAR clustering + sensor fusion. The simulator provides raw camera images and LiDAR scans instead of pre-computed cone positions:

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup autonomy.launch.py
```

Drive 5 laps then stop:

```bash
ros2 launch falcon_bringup autonomy.launch.py total_laps:=5
```

Override other arguments:

```bash
ros2 launch falcon_bringup autonomy.launch.py total_laps:=3 gazebo_gui:=false rviz:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `total_laps` | `0` | Number of laps to drive (0 = unlimited) |
| `gazebo_gui` | `true` | Show Gazebo 3D window |
| `rviz` | `true` | Launch RViz |
| `launch_group` | `default` | EUFS sensor group (`default` = cameras + LiDAR) |

### Architecture (perception mode)

```
EUFS Gazebo (cameras + LiDAR)
  │
  ├─ /zed/image_raw ──→ YOLO ──→ /yolo/detections
  │                                    │
  ├─ /zed/depth/image_raw ──→ cone_depth_localizer ──→ /falcon/camera_cones
  │                                                          │
  ├─ /gazebo_scan ──→ lidar_cone_detector ──→ /falcon/lidar_cones
  │                                                │
  │                              cone_fuser ◄──────┘
  │                                 │
  │                 │
  │     /falcon/fused_cones (eufs_msgs)
  │                 │
  │   cone_map_builder ── /map/cone_map ──► path_planner ◄───────┘
  │                                                │
  │                                         /planning/path
  │                                                │
  │                                         pure_pursuit
  │                                                │
  └──────── /cmd ◄─────────────────────────────────┘
```

### Architecture (ground-truth dev mode)

**1. Map Builder Node**
```
EUFS Gazebo                         
  └─ /cones ───► cone_map_builder ──► /map/cone_map
  (eufs_msgs)                          (map frame)
```

**2. Planning Nodes**
```
EUFS Gazebo                         
  └─ /cones ───┬─► path_planner ──┐
  (eufs_msgs)  │                  │
/map/cone_map ─┘           /planning/path
                                  │
                           pure_pursuit
                                  │
                              /cmd ──► EUFS
```

### How it works

1. **Perception stack** (perception mode) or **EUFS `/cones`** (ground-truth mode)
   provides cone detections in the vehicle body frame (`base_footprint`).

2. **cone_map_builder** subscribes to cone observations, transforms each one into
   the `map` frame via TF, and accumulates a Kalman-filtered landmark map. After the
   first lap, this map contains *every cone on the track* — even those currently behind
   the car.

3. **path_planner** subscribes to *both* sources:
   - `/falcon/fused_cones` (or `/cones` in dev mode) — live cones, body frame.
   - `/map/cone_map` — accumulated map cones, transformed from map → base_footprint
     via TF at each planning cycle.

   It **deduplicates** (merging map and live cones), then computes a midpoint centerline:
   - Separates cones into blue (left) and yellow (right) boundary lists.
   - For each cone, finds its nearest counterpart on the opposite side to compute a midpoint.
   - Orders all midpoints with a greedy nearest-neighbour traversal starting from the car.
   - First lap: builds the map. Subsequent laps: the map has the complete track, helping navigate blind spots.

4. **Lap counting** — the planner tracks orange/big-orange cones near the car (within
   `orange_detect_radius_m`). When the car enters then exits the orange zone, it
   increments the lap counter (the initial departure from the start line is ignored).
   After `total_laps` (0 = unlimited), the planner stops publishing paths, which causes
   pure_pursuit's path-timeout safety to bring the car to a stop.

5. **pure_pursuit** follows `/planning/path` with Ackermann steering commands on `/cmd`.
   - Locates the **lookahead point**: the nearest path waypoint strictly ahead of the car that is ≥ `lookahead_distance` away.
   - Computes turning angle `alpha` to that point, then calculates the required steering `delta = atan(2 * wheelbase * sin(alpha) / lookahead_distance)`.

### RViz visualisation

RViz opens automatically. Set **Fixed Frame = `base_footprint`** (for live view) or
`map` (for accumulated map) and add:

| Display type | Topic | What you see |
|---|---|---|
| MarkerArray | `/planning/cone_markers` | Blue/yellow/orange cones + green centerline |
| MarkerArray | `/map/cone_markers` | Accumulated map cones with covariance ellipses |
| Marker | `/planning/lookahead_marker` | Orange sphere — current steering target |
| Path | `/planning/path` | Waypoints as arrows |
| MarkerArray | `/falcon/cone_markers` | Camera-detected cones (perception mode) |
| MarkerArray | `/falcon/lidar_cluster_markers` | LiDAR cluster centroids (perception mode) |

### Tuning

```bash
ros2 param set /pure_pursuit_node lookahead_distance 3.5   # smoother, less oscillation
ros2 param set /pure_pursuit_node target_speed 1.5         # slower if car spins out
```

### Path planner parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cones_topic` | `/falcon/fused_cones` | Live FOV cones (body frame) |
| `map_topic` | `/map/cone_map` | Accumulated landmark map (map frame) |
| `total_laps` | `0` | Laps to drive (0 = unlimited) |
| `orange_detect_radius_m` | `5.0` | Distance within which orange cones trigger lap counting |
| `dedup_radius_m` | `1.0` | Merge radius for map vs live cone deduplication |
| `waypoint_ordering` | `forward_x` | `forward_x` (sort by +x) or `nearest_neighbor` |
| `path_extend_m` | `3.0` | Extend path past last midpoint (metres; 0 = off) |
| `min_cones_per_side` | `1` | Minimum blue *and* yellow cones required to plan |

### Pure pursuit parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lookahead_distance` | `2.5` | Lookahead circle radius (metres) |
| `wheelbase` | `1.53` | Vehicle wheelbase (metres) |
| `target_speed` | `2.0` | Target forward speed (m/s) |
| `max_steering_angle` | `0.44` | Steering clamp (radians, ~25°) |
| `path_timeout_sec` | `0.5` | Stop if no path received within this window |

*Tuning pure pursuit:* If the car weaves, increase `lookahead_distance`. If it cuts corners, decrease it. If spinning out on turns, decrease `target_speed`.

---

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `falcon_msgs` | C++ | Cone and ConeArray message definitions |
| `falcon_bringup` | Python | Launch files |
| `falcon_common` | C++ | Shared utilities |
| `falcon_teleop` | Python | Keyboard teleop with embedded front-view |
| `falcon_drivers` | — | ZED/LiDAR config placeholders |
| `falcon_cone_perception` | C++ | Raw cone detection from pointcloud + YOLO + fusion |
| `falcon_cone_map_builder` | C++ | Global cone map |
| `falcon_localization` | C++ | Localization |
| `falcon_planning` | Python | Centerline path planner + Pure Pursuit controller |
| `falcon_vehicle_comm` | C++ | Vehicle interface |

## Teleop camera launch

The teleop window shows a front camera view from a dedicated **teleop camera** Not the ZED but it is a separate minimal Gazebo camera at the same mount position, always on in any launch group.

**Launch order:**

1. Start simulation:
   ```bash
   ros2 launch falcon_bringup simulation.launch.py
   ```

2. In a separate terminal, launch teleop (with camera view):
   ```bash
   ros2 launch falcon_teleop teleop.launch.py
   ```

**Topics:** `/teleop_camera/image_raw`, `/teleop_camera/camera_info`  
**Dependencies:** `cv_bridge`, `opencv-python`, `Pillow` (required for teleop camera display)
