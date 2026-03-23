# falcon_cone_bridge

Bridges EUFS simulator cone ground-truth into the Falcon perception pipeline.

Subscribes to `/cones` (`eufs_msgs/ConeArrayWithCovariance`) published by the EUFS Gazebo plugin and republishes it as `/perception/cones_raw` (`falcon_msgs/ConeArray`), allowing the planning stack to run without a working LiDAR or camera detector.

## Why this exists

The EUFS simulator's `ConeGroundTruthPlugin` publishes the exact positions of every cone visible to the car's sensors on `/cones`. This node converts that message type to the Falcon `ConeArray` format so the rest of the stack (`cone_fusion_node` → `path_planner_node` → `pure_pursuit_node`) receives data in its expected format.

## Topic flow

```
EUFS sim
  └─ /cones  (eufs_msgs/ConeArrayWithCovariance, base_footprint frame)
       └─ [cone_bridge_node]
            └─ /perception/cones_raw  (falcon_msgs/ConeArray)
                 └─ cone_fusion_node
                      └─ /perception/cones_fused  (falcon_msgs/ConeArray)
                           └─ path_planner_node
```

## Design notes

- **No TF transform** — cones are forwarded in `base_footprint` (vehicle frame) exactly as the simulator publishes them. The planning stack operates entirely in vehicle frame so no odometry or TF tree is required.
- Covariance is propagated from the eufs 4-element format (`xx, xy, yx, yy`) into the 36-element ROS covariance matrix.

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/cones` | EUFS source topic |
| `output_topic` | `/perception/cones_raw` | Falcon output topic |

## Launch

Standalone test:
```bash
ros2 launch falcon_cone_bridge cone_bridge.launch.py
```

Full autonomy stack:
```bash
ros2 launch falcon_bringup autonomy.launch.py
```

## Verify

```bash
ros2 topic hz /cones /perception/cones_raw  # both should match (~10-20 Hz)
ros2 topic echo /perception/cones_raw --once
```
