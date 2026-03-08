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

## Run

### Full stack (real sensors)

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup bringup.launch.py
```

### Simulation (EUFS Gazebo)

```bash
source scripts/setup_falcon.sh
ros2 launch falcon_bringup simulation.launch.py
```

Defaults: small track, ads-dv robot, velocity control, dry track, ground truth TFs, Gazebo GUI, RViz, RQT steering. Override with `gazebo_gui:=false` or `rviz:=false` if needed.

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

## Topic Conventions (real sensors)

| Topic                   | Type             | Description                                      |
|-------------------------|------------------|--------------------------------------------------|
| `/perception/cones_raw` | falcon_msgs/ConeArray | Raw detections (base_link or sensor frame)      |
| `/perception/cones_fused` | falcon_msgs/ConeArray | Locally-fused cones (odom frame)              |
| `/map/cones_map`        | falcon_msgs/ConeArray | Mapped/global cones (map frame)                 |

Use `header.frame_id` to declare the frame.

## Frame Conventions

| Stage   | Frame       |
|---------|-------------|
| Raw     | base_link (preferred) or sensor optical frame |
| Fused   | odom        |
| Map     | map         |

## Packages

- `falcon_msgs` – Cone and ConeArray message definitions
- `falcon_bringup` – Launch files
- `falcon_common` – Shared C++ utilities 
- `falcon_drivers` – ZED/LiDAR config placeholders
- `falcon_cone_perception` – Raw cone detection from pointcloud
- `falcon_cone_fusion` – Local cone fusion
- `falcon_cone_map_builder` – Global cone map
- `falcon_localization` – Localization 
- `falcon_planning` – Planning
- `falcon_vehicle_comm` – Vehicle interface 
