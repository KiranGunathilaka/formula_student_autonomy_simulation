# Lidar profiles

One file per lidar model. `ros2 launch falcon_bringup sim.launch.py lidar:=<file name>`
(or `lidar_file:=/any/path.yaml`). Copy a file to try another channel layout.

```yaml
name: Hesai ATX
kind: solid_state            # solid_state | spinning (only changes the housing drawn)
rate_hz: 10
horizontal: {fov_deg: 120, samples: 1500}   # columns per frame over the view
vertical:                                   # either an even fan ...
  uniform: {min_deg: -10, max_deg: 10, channels: 256}
  # ... or every channel's elevation in degrees:
  # angles_deg: [-55, -52, ..., 13, 15]
range: {min_m: 0.3, max_m: 100}
noise_stddev_m: 0.02
```

An uneven `angles_deg` table is simulated exactly: it is split into runs of equal
spacing, one Gazebo ray sensor per run, and `lidar_merger` joins them into one cloud.
`ros2 run falcon_description falcon_urdf --lidar <name>` prints the split.

Mount (height, pitch) lives in the vehicle file, not here, because the same lidar
sits differently on each car; override it per run with `lidar_z:=`, `lidar_pitch_deg:=`.
