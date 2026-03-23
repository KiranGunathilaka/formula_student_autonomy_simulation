Launch files for the Falcon autonomy stack.

| Launch file | Description |
|-------------|-------------|
| `simulation.launch.py` | EUFS Gazebo sim only (small track, ads-dv, simulated cones on `/cones`) |
| `autonomy.launch.py` | Full autonomous stack: sim + cone bridge + cone fusion + path planner + pure pursuit |
| `bringup.launch.py` | Full stack with real sensors (cone perception, fusion, map builder) |

## autonomy.launch.py

Runs the car autonomously in simulation using ground-truth cone positions.

```bash
ros2 launch falcon_bringup autonomy.launch.py
```

Node graph:
```
EUFS sim → /cones
  └─ cone_bridge_node  → /perception/cones_raw
       └─ cone_fusion_node  → /perception/cones_fused
            └─ path_planner_node  → /planning/path
                 └─ pure_pursuit_node  → /cmd
```

See the top-level README for the full launch guide and RViz setup.
