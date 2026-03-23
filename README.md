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

Run the full planning and control stack against the EUFS simulator using ground-truth cone positions — no LiDAR or camera detection required:

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup autonomy.launch.py
```

Drive 3 laps then stop:

```bash
ros2 launch falcon_bringup autonomy.launch.py total_laps:=3
```

### Architecture

```
EUFS Gazebo                                     ┌──────────────────────┐
  └─ /cones ─────────┬──── cone_bridge ─────┐   │                      │
     (base_footprint) │      └─ /perception/ │   │   PATH PLANNER       │
                      │         cones_raw    │   │                      │
                      │           │          │   │  ┌─ live cones ────┐ │
                      │      cone_fusion     │   │  │  (body frame)   │ │   /planning/path
                      │           │          ├───┤  │                 ├─┼──────────────┐
                      │    /perception/      │   │  │  merge + dedup  │ │              │
                      │     cones_fused ─────┘   │  │                 │ │       pure_pursuit
                      │                          │  │                 │ │              │
                      └── cone_map_builder       │  │  map cones      │ │          /cmd
                           │                     │  │  (map→body TF)  │ │              │
                      /map/cone_map ─────────────┤  └─────────────────┘ │         EUFS sim
                       (map frame)               │                      │
                                                 │  lap counter         │
                                                 │  (orange proximity)  │
                                                 └──────────────────────┘
```

### How it works

1. **EUFS publishes `/cones`** (base_footprint frame) — only cones in the sensor FOV
   (front semicircle), not the full track.

2. **cone_bridge** converts EUFS messages to Falcon format on `/perception/cones_raw`.
   **cone_fusion** relabels to `/perception/cones_fused`. These give the planner
   *live field-of-view cones* in the vehicle body frame.

3. **cone_map_builder** also subscribes to `/cones`, transforms each observation into
   the `map` frame via TF, and accumulates a Kalman-filtered landmark map. After the
   first lap, this map contains *every cone on the track* — even those currently behind
   the car.

4. **path_planner** subscribes to *both* sources:
   - `/perception/cones_fused` — live cones, already in body frame.
   - `/map/cone_map` — accumulated map cones, transformed from map → base_footprint
     via TF at each planning cycle.

   It **deduplicates** (if a map cone is within `dedup_radius_m` of a live cone of the
   same color, the live one wins) and then runs the midpoint planning algorithm on the
   merged set. This means:
   - **First partial lap:** only live FOV cones are available; the car may see only
     one side of the track (e.g. 5 blue, 0 yellow). As more cones are observed and
     added to the map, both sides become visible.
   - **After one full lap:** the map has the complete track, so the planner always
     has blue *and* yellow cones to pair — even in sections where live FOV only
     shows one side.

5. **Lap counting** — the planner tracks orange/big-orange cones near the car (within
   `orange_detect_radius_m`). When the car enters then exits the orange zone, it
   increments the lap counter. After `total_laps` (0 = unlimited), the planner stops
   publishing paths, which causes pure_pursuit's path-timeout safety to bring the car
   to a stop.

6. **pure_pursuit** follows `/planning/path` with Ackermann steering commands on `/cmd`.

### RViz visualisation

RViz opens automatically. Set **Fixed Frame = `base_footprint`** and add:

| Display type | Topic | What you see |
|---|---|---|
| MarkerArray | `/planning/cone_markers` | Blue/yellow/orange cones + green centerline |
| MarkerArray | `/map/cone_markers` | Accumulated map cones with covariance ellipses |
| Marker | `/planning/lookahead_marker` | Orange sphere — current steering target |
| Path | `/planning/path` | Waypoints as arrows |

### Tuning

```bash
ros2 param set /pure_pursuit_node lookahead_distance 3.5   # smoother, less oscillation
ros2 param set /pure_pursuit_node target_speed 1.5         # slower if car spins out
```

### Path planner parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cones_topic` | `/perception/cones_fused` | Live FOV cones (body frame) |
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

See [`falcon_planning/README.md`](falcon_ws/src/falcon_planning/README.md) for the full parameter reference and tuning guide.

---

## Packages

| Package | Type | Description |
|---------|------|-------------|
| `falcon_msgs` | C++ | Cone and ConeArray message definitions |
| `falcon_bringup` | Python | Launch files |
| `falcon_common` | C++ | Shared utilities |
| `falcon_teleop` | Python | Keyboard teleop with embedded front-view |
| `falcon_drivers` | — | ZED/LiDAR config placeholders |
| `falcon_cone_perception` | C++ | Raw cone detection from pointcloud |
| `falcon_cone_fusion` | C++ | Local cone fusion |
| `falcon_cone_map_builder` | C++ | Global cone map |
| `falcon_localization` | C++ | Localization |
| `falcon_cone_bridge` | Python | Converts EUFS sim cones → Falcon ConeArray (sim only) |
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
