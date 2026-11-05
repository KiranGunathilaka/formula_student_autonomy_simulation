# falcon_autonomy

Formula Student Driverless stack on the EUFS Gazebo simulator: cone perception
(camera + lidar), a cone map, a centreline planner and pure pursuit. It runs on the
IMechE ADS-DV (the FS-AI DDT car) or EUFS's own FS car, with a choice of lidar.

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

Or in Docker: `docker/docker_build.sh`, then `docker/docker_run.sh` (mounts `falcon_ws/src`,
and `runs/` for coverage reports).

## Run the simulation: one command

```bash
ros2 launch falcon_bringup sim.launch.py                       # ADS-DV + Hesai ATX, autonomy
ros2 launch falcon_bringup sim.launch.py vehicle:=eufs         # EUFS's FS car instead
ros2 launch falcon_bringup sim.launch.py lidar:=ouster_os1_32 lidar_z:=0.70
ros2 launch falcon_bringup sim.launch.py mode:=teleop          # drive it yourself
ros2 launch falcon_bringup sim.launch.py mode:=both            # autonomy, W/A/S/D take over
ros2 launch falcon_bringup sim.launch.py cones:=perception     # drive on YOLO + lidar fusion
ros2 launch falcon_bringup sim.launch.py track:=comp_2021 laps:=2 target_speed:=3
ros2 launch falcon_bringup sim.launch.py --show-args           # every argument
```

| Argument | Default | |
|---|---|---|
| `vehicle` | `ads_dv` | `ads_dv` or `eufs`, or a path to your own vehicle YAML |
| `lidar` | `hesai_atx` | a lidar profile name (below) or a path to a YAML |
| `track` | `small_track` | any `eufs_tracks` world |
| `mode` | `autonomy` | `autonomy`, `teleop`, `both`, `none` |
| `cones` | `sim` | `sim`: drive on the simulator's cones (`/cones`); `perception`: on `/falcon/fused_cones` |
| `laps` | `0` | laps for the planner, 0 = unlimited |
| `target_speed` | `3.0` | pure pursuit top speed, m/s |
| `use_lidar`, `use_camera` | `true` | leave a sensor out (faster sim) |
| `lidar_x/_y/_z/_roll_deg/_pitch_deg/_yaw_deg` | from the vehicle | lidar mount for this run |
| `camera_x/_y/_z/_roll_deg/_pitch_deg/_yaw_deg` | from the vehicle | camera mount for this run |
| `lidar_sensor_type` | `gpu_ray` | `ray` raycasts on the CPU: exact but slow (see Troubleshooting) |
| `lidar_h_samples` | profile | fewer columns per frame, for a slow PC |
| `coverage` | `false` | run the cone coverage metric (below) |
| `coverage_source` | `raycast` | `raycast` (exact rays from the true pose) or `pointcloud` (Gazebo's lidar) |
| `gazebo_gui`, `rviz` | `true` | |

Mount convention: `x` is metres ahead of the middle of the wheelbase, `z` metres above
the ground, `pitch_deg` > 0 tilts the sensor down.

The old entry points still work and now call `sim.launch.py`:
`simulation.launch.py` (nothing driving; `launch_group:=default` turns the lidar and
camera on) and `autonomy.launch.py total_laps:=5` (= `mode:=autonomy cones:=perception`).

### Switching car and moving sensors

Vehicle profiles live in `falcon_ws/src/falcon_description/config/vehicles/`:

| Profile | Car | Wheelbase | Steering limit | Lidar mount | Camera mount |
|---|---|---|---|---|---|
| `ads_dv` | IMechE ADS-DV (DDT) | 1.530 m | ±21° (what the AI may request) | sensor plate, 0.423 m ahead, 0.57 m up | plate, 0.62 m up, 15° down |
| `eufs` | EUFS FS car | 1.580 m | 0.52 rad | front wing, 0.25 m up | above the driver, 1.01 m up |

Each file says where its numbers come from. To move a sensor for good, edit the
`sensors:` block of the vehicle file (with `--symlink-install` no rebuild is needed);
for one run, use the `lidar_*` / `camera_*` launch arguments. The URDF is generated from
these files at launch, and every node that needs a sensor pose (lidar detector, camera
localiser, coverage) reads it from TF, so nothing else has to change.

Check a set-up without starting Gazebo:

```bash
ros2 run falcon_description falcon_urdf --list
ros2 run falcon_description falcon_urdf --vehicle ads_dv --lidar robosense_helios_5515 --summary --set lidar_z=0.7
```

### Lidar profiles

In `falcon_ws/src/falcon_description/config/lidars/` (see the README there for the format):

| Profile | Channels | Vertical | Horizontal |
|---|---|---|---|
| `hesai_atx` | 256 | ±10° (even) | 120°, 1500 columns |
| `velodyne_vlp16` | 16 | ±15°, 2° apart | 360°, 0.2° |
| `robosense_helios_1615` | 32 | −16°..+15°, 1° apart | 360°, 0.2° |
| `robosense_helios_5515` | 32 | −55°..+15°, uneven | 360°, 0.2° |
| `hesai_xt32` | 32 | −16°..+15°, 1° apart | 360°, 0.18° |
| `ouster_os1_32` / `_64` / `_128` | 32 / 64 / 128 | ±21.2° (even) | 360°, 2048 columns |
| `ouster_os2_128` | 128 | ±11.25° (even) | 360°, 2048 columns |
| `example_64_gradient` | 64 | example uneven layout, dense near the horizon | 360° |

A profile gives either an even fan (`uniform: {min_deg, max_deg, channels}`) or every
channel's elevation (`angles_deg: [...]`), so any vertical-resolution layout can be
tried by copying a file. Gazebo only spaces a ray sensor's channels evenly, so an uneven
table is split into evenly spaced runs, one Gazebo ray sensor each (the Helios 5515
becomes 4), and `lidar_merger` joins them into one cloud. The cloud is always on
`/lidar/points` (frame `lidar_link`).

## Cone coverage: did the lidar see every cone?

```bash
ros2 launch falcon_bringup sim.launch.py lidar:=hesai_atx coverage:=true
```

`cone_coverage` counts the lidar returns on every cone of the track (from the
simulator's ground truth) in every lidar frame, and after `coverage_laps` laps
(default 1) writes a report to `~/falcon_runs/<run>/` (`coverage_output_dir:=`, or
`$FALCON_RUNS_DIR`).

How the returns are counted (`coverage_source:=`):

- `raycast` (default): the node casts the lidar profile's own rays (every channel ×
  column, at the profile's rate) from the car's true pose against the cones, the
  flat ground and the car's body (`occluder_box` in the vehicle file). Cones are
  frustums of the FS rulebook size (`coverage_cone_shape:=fs`, 325 / 505 mm, as in
  `lidar_coverage_and_hairpins.html`) or the Gazebo meshes (`:=eufs`). It is exact,
  cheap, and does not need the lidar in Gazebo at all (`use_lidar:=false` runs much
  faster). Against Gazebo's exact CPU `ray` sensor on the EUFS meshes it gave the same
  detected / not-detected answer for 39 of 44 cones in view, with about 1.5× more
  points per cone because the meshes are faceted and the frustums smooth.
- `pointcloud`: counts the points of Gazebo's `/lidar/points`. Use it to check the
  simulated sensor itself, on a machine where Gazebo's lidar renders correctly (see
  Troubleshooting).

The report:

- `summary.json`: cones detected / with too few points / in view with no points / never
  in view, per colour, the range at which cones were first detected, and the share of
  cones detected in each range band (0–5 m, 5–10 m, ...);
- `cones.csv`: one row per cone (status, most points and channels in one scan, ranges);
- `run.json`: vehicle, lidar profile, mount and criteria.

A cone counts as detected when one scan put at least `coverage_min_points` (3) points on
it from at least `coverage_min_rings` (1) channel; use `coverage_min_rings:=2` for a
stricter test. While it runs, `/coverage/summary` has a one-line status (also shown in
the teleop window) and `/coverage/markers` colours every cone in RViz: green detected,
orange too few points, red in view but no points, grey never in view. The report can
be written at any time with `ros2 service call /coverage/write_report std_srvs/srv/Trigger`.

Drive the lap however you like: autonomy (default), `mode:=teleop` (press C for cruise
and steer with A/D), or `mode:=both`.

### Comparing lidars and mounts (sweeps)

```bash
ros2 run falcon_evaluation lidar_sweep \
  $(ros2 pkg prefix falcon_evaluation)/share/falcon_evaluation/config/sweep_example.yaml
```

Runs one headless lap per set-up (camera and Gazebo lidar off, coverage ray-cast,
driving on the simulator's cones so the path does not depend on the lidar under test)
and writes `sweep_summary.csv` / `.md` with one row per run. A sweep file lists runs and/or a grid of any `sim.launch.py`
arguments, e.g. `grid: {lidar: [ouster_os1_32, robosense_helios_1615], lidar_z: [0.57, 0.70], lidar_pitch_deg: [0, 2]}`.
`--dry-run` prints the commands; `--table <dir>` re-tabulates finished runs.

## Teleop

`mode:=teleop` or `mode:=both` opens the window with a chase camera; on a running
simulation, `ros2 launch falcon_teleop teleop.launch.py` does the same.

| Key | Action |
|---|---|
| W / ↑ | accelerate to the cruise speed (hold) |
| S / ↓ | brake |
| A / D, ← / → | steer (hold) |
| C | cruise on/off: keep the cruise speed without holding W |
| R / F | cruise speed ± 0.5 m/s |
| Space | stop |
| M | manual ↔ auto |

Click the window first so it has keyboard focus. Commands go through `cmd_mux`: the
keyboard has priority, and in AUTO the planner drives. Any driving key takes over
from the planner; M hands back.

## Path planning and control

**Planner** (`falcon_planning/centerline.py`): cones within 30 m of the car (live cones
plus the map) are triangulated (Delaunay). The path walks from the car through
triangles, from the midpoint of one blue–yellow edge to the next; same-colour edges
are walls, so the path cannot jump to a neighbouring part of the track however close
it is. If only one side is in view, each boundary is chained from the car forward and
paired with a virtual cone 3 m across. The path is smoothed, resampled at 0.5 m and
published in the `map` frame.

**Pure pursuit** (`falcon_planning/pursuit.py`): every control step the path is moved
into the car frame with TF, the lookahead point is the exact point where the path
leaves a circle of radius max(2.5 m, 0.6 s × speed), steering is computed from the
rear axle, and speed drops where the path ahead is tight (`max_lateral_accel`).

Why it changed: the old planner paired every yellow cone with its nearest blue cone
over the whole map and ordered the midpoints nearest-neighbour. Corners (more cones on
the outside) gave diagonal pairs and zig-zags, once the map held the whole track cones
were paired across parts of the track, and the pairing took ~0.5 s per cycle on the
984-cone hairpin track, so the 10 Hz planner fell behind and pure pursuit timed out.
Pure pursuit also aimed at the nearest waypoint beyond 2.5 m anywhere on the path, so
it cut corners.

Tracks with 9 m hairpins (`comp_2021`, `garden_light`, `hairpins_increasing_difficulty`)
are tighter than the ADS-DV's turning circle at ±21° (about 10.1 m are needed), so it
touches cones there whatever the planner does; the EUFS car gets round.

### Planner parameters (`config/path_planner.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `cones_topic` | `/falcon/fused_cones` | live cones (body frame); `/cones` with `cones:=sim` |
| `map_topic` | `/map/cone_map` | accumulated map (map frame) |
| `window_radius_m` | `30.0` | cones further from the car are ignored |
| `track_width_m` | `3.0` | virtual cone offset when one side is not seen |
| `max_track_width_m` | `6.0` | wider blue–yellow gaps are not the track |
| `total_laps` | `0` | laps to drive (0 = unlimited) |
| `orange_detect_radius_m` | `3.0` | orange cones this close count as the start/finish zone |

### Pure pursuit parameters (`config/pure_pursuit.yaml`)

| Parameter | Default | Description |
|---|---|---|
| `lookahead_distance` | `2.5` | minimum lookahead (m) |
| `lookahead_gain` | `0.6` | lookahead grows with speed: max(distance, gain × speed) |
| `target_speed` | `3.0` | top speed (m/s) |
| `max_lateral_accel` | `3.0` | slows down where the path ahead is tight (0 = off) |
| `path_timeout_sec` | `0.5` | stop if no path within this (sim time) |
| `wheelbase`, `max_steering_angle` | from the vehicle profile | |

If the car weaves, increase `lookahead_distance`; if it cuts corners, decrease it.

## Topics

| Topic | Type | |
|---|---|---|
| `/lidar/points` | PointCloud2 (best effort) | lidar cloud, frame `lidar_link` |
| `/zed/image_raw`, `/zed/depth/image_raw` | Image | camera (RGB and depth) |
| `/teleop_camera/image_raw` | Image | chase camera (teleop only) |
| `/cones` | eufs_msgs/ConeArrayWithCovariance | simulated cones (FOV 180°, 20 m; colour within the camera view, 15 m) |
| `/falcon/lidar_cones`, `/falcon/camera_cones`, `/falcon/fused_cones` | eufs_msgs/ConeArrayWithCovariance | perception |
| `/map/cone_map` | falcon_msgs/ConeArray | cone map |
| `/planning/path` | nav_msgs/Path | centreline (map frame) |
| `/falcon/cmd_auto`, `/falcon/cmd_teleop` → `/cmd` | AckermannDriveStamped | via `cmd_mux` |
| `/falcon/control_source` | String | `auto`, `teleop` or `idle` |
| `/ground_truth/state`, `/ground_truth/track` | eufs_msgs | simulator truth |
| `/coverage/summary`, `/coverage/markers` | String, MarkerArray | coverage metric |

TF: `map → base_footprint` from the simulator (exact; EUFS's pose noise is zero), then
the URDF (`base_footprint` is on the ground under the middle of the wheelbase).

## Packages

| Package | Type | Description |
|---|---|---|
| `falcon_bringup` | Python | `sim.launch.py` and the older launch files, RViz layout |
| `falcon_description` | Python | vehicle and lidar profiles, URDF builder, `falcon_urdf` tool |
| `falcon_evaluation` | Python | cone coverage metric, `lidar_sweep` |
| `falcon_cone_perception` | CMake/Python | YOLO + depth localiser, 3D/2D lidar cone detectors, fuser, lidar merger |
| `falcon_cone_map_builder` | C++ | global cone map |
| `falcon_planning` | Python | centreline planner + pure pursuit |
| `falcon_teleop` | Python | keyboard teleop, `cmd_mux` |
| `falcon_msgs` | C++ | Cone and ConeArray messages |
| `falcon_common`, `falcon_localization`, `falcon_vehicle_comm`, `falcon_drivers` | | placeholders |

## Troubleshooting

### gzserver exits with code 255

1. **Use `setup_falcon.sh`** so `EUFS_MASTER` and the install paths are set.
2. **Permission issues**: if Gazebo or the build ran with `sudo`, fix ownership:
   ```bash
   sudo chown -R $USER:$USER ~/.gazebo ~/.ros
   sudo chown -R $USER:$USER falcon_ws/install   # if install is root-owned
   ```
3. **GPU/graphics**: try `gazebo_gui:=false rviz:=false` first. The lidar (`gpu_ray`) and
   cameras render with OpenGL even headless, so gzserver needs a display (`DISPLAY`) and
   a GL driver. Without one, use `use_lidar:=false use_camera:=false` (coverage still
   works) or `lidar_sensor_type:=ray` with a sparse lidar.

### Gazebo's lidar returns too few points

Check before trusting `/lidar/points` (e.g. for `cones:=perception` or
`coverage_source:=pointcloud`): with the car parked, a VLP-16 at 0.70 m should give
about 15,000 points a frame, with all 8 downward rings complete
(`ros2 topic echo --once --field width /lidar/points`).

On the test machine (Intel UHD 630 through Mesa, NVIDIA driver not loaded) the
`gpu_ray` sensor returned only a fraction of that: 947 of ~15,000 for the VLP-16,
almost no ground and nothing past 5 m, and about a tenth of the expected returns for
the ATX. The CPU `ray` sensor (`lidar_sensor_type:=ray`) was exact but ran at 0.17×
real time with the VLP-16 and ran out of memory (3.7 GB) with the ATX. Whether
`gpu_ray` is right on an NVIDIA GPU was not tested. This is why coverage defaults to
the ray-cast, which does not depend on Gazebo's lidar.

### The sim runs slowly

`use_camera:=false` saves the most, then `use_lidar:=false` (coverage still works by
ray-cast). With a Gazebo lidar: `lidar_h_samples:=` (e.g. 750 for the ATX) or
`lidar_rate_hz:=5`.
