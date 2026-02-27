# falcon_autonomy

Formula Student Driverless stack: ZED1 + 2D LiDAR, cone-based mapping.

**ROS 2 Humble.**

## Build

From the repository root:

```bash
. ./scripts/rebuild_ws.sh
```

Don't run , source it. Also this will change after containerization is done. Then update

## Run

Full stack (real sensors):

```bash
source falcon_ws/install/setup.sh
ros2 launch falcon_bringup bringup.launch.py use_sim:=false
```

Full stack with simulated cone publishers (no pointcloud needed):

```bash
source falcon_ws/install/setup.sh
ros2 launch falcon_bringup bringup.launch.py use_sim:=true
```

Simulation only:

```bash
source falcon_ws/install/setup.sh
ros2 launch falcon_bringup sim_only.launch.py
```

## Topic Conventions

| Topic                   | Type             | Description                                      |
|-------------------------|------------------|--------------------------------------------------|
| `/perception/cones_raw` | falcon_msgs/ConeArray | Raw detections (base_link or sensor frame)      |
| `/perception/cones_fused` | falcon_msgs/ConeArray | Locally-fused cones (odom frame)                |
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
- `falcon_sim` – Simulation dummy publishers and assets
