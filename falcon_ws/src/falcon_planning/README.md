# falcon_planning

Planning and control stack for the Falcon autonomy car.

Contains two nodes:
- **`path_planner_node`** — computes a centerline path between blue and yellow cones
- **`pure_pursuit_node`** — tracks that path using the Pure Pursuit algorithm and drives the car

## Architecture

Everything runs in the vehicle body frame (`base_footprint`). The car is always at the origin (0, 0) facing +x, so no odometry or TF lookups are required.

```
/perception/cones_fused  (falcon_msgs/ConeArray, base_footprint frame)
    └─ path_planner_node
         └─ /planning/path          (nav_msgs/Path)
         └─ /planning/cone_markers  (visualization_msgs/MarkerArray)
              └─ pure_pursuit_node
                   └─ /cmd  (ackermann_msgs/AckermannDriveStamped)
                   └─ /planning/lookahead_marker  (visualization_msgs/Marker)
```

---

## path_planner_node

Computes a midpoint centerline path from cone positions.

### Algorithm

1. Separate cones into blue (left boundary) and yellow (right boundary) lists.
2. For each yellow cone, find its nearest blue counterpart and compute the midpoint.
3. Unpaired blue cones are also given a midpoint with their nearest yellow.
4. Order all midpoints with a greedy nearest-neighbour traversal starting from the car's position (origin).
5. Publish the ordered list as `nav_msgs/Path` and coloured sphere markers for RViz.

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cones_topic` | `/perception/cones_fused` | Input cone topic |
| `path_topic` | `/planning/path` | Output path topic |
| `markers_topic` | `/planning/cone_markers` | RViz markers topic |
| `min_cones_per_side` | `1` | Minimum cones required on each side before planning |
| `plan_rate_hz` | `10.0` | Planning frequency (Hz) |

### RViz markers

| Namespace | Colour | Meaning |
|-----------|--------|---------|
| `blue_cones` | Blue | Blue cone positions |
| `yellow_cones` | Yellow | Yellow cone positions |
| `waypoints` | Green | Computed centerline midpoints |
| `path_line` | Green line | Path connecting all waypoints |

---

## pure_pursuit_node

Geometric path tracking controller that outputs Ackermann steering commands.

### Algorithm

1. From the latest path, find the **lookahead point**: the nearest waypoint at distance ≥ `lookahead_distance` from the origin (car position) that has `x > 0` (ahead of the car).
2. Compute the heading error:
   ```
   alpha = atan2(waypoint.y, waypoint.x)
   ```
3. Compute the steering angle:
   ```
   delta = atan(2 * wheelbase * sin(alpha) / L)
   ```
   where `L` is the actual distance to the lookahead point.
4. Clamp to `[-max_steering_angle, +max_steering_angle]` and publish with `target_speed`.

### Safety

- If no path arrives within `path_timeout_sec`, the car stops.
- If the last waypoint is within `goal_tolerance_m` of the origin, the car stops (path complete).

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `path_topic` | `/planning/path` | Input path topic |
| `cmd_topic` | `/cmd` | Output Ackermann command topic |
| `lookahead_distance` | `2.5` | Lookahead distance L (metres) |
| `wheelbase` | `1.53` | Vehicle wheelbase (metres, ADS-DV) |
| `target_speed` | `2.0` | Forward speed (m/s) |
| `max_steering_angle` | `0.44` | Steering clamp (radians, ~25°) |
| `control_rate_hz` | `20.0` | Control loop frequency (Hz) |
| `path_timeout_sec` | `0.5` | Stop if path is older than this |
| `goal_tolerance_m` | `1.0` | Stop when within this distance of last waypoint |

### Tuning guide

| Symptom | Fix |
|---------|-----|
| Car oscillates / weaves | Increase `lookahead_distance` |
| Car cuts corners | Decrease `lookahead_distance` |
| Car too slow | Increase `target_speed` (test incrementally) |
| Car spins out on turns | Decrease `target_speed` or increase `lookahead_distance` |

---

## Launch

```bash
# Planning nodes only (requires perception pipeline running separately)
ros2 launch falcon_planning planning.launch.py

# Full autonomy stack (sim + bridge + fusion + planning)
ros2 launch falcon_bringup autonomy.launch.py
```

## Live parameter tuning

```bash
ros2 param set /pure_pursuit_node lookahead_distance 3.5
ros2 param set /pure_pursuit_node target_speed 1.5
```

## Verify

```bash
ros2 topic hz /planning/path /cmd           # path ~10Hz, cmd ~20Hz
ros2 topic echo /planning/path --once       # check waypoints exist
ros2 topic echo /cmd --once                 # check speed/steering non-zero
```
