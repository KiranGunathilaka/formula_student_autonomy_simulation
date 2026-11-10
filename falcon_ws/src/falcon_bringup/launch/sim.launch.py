"""
Falcon simulation: pick the car, the lidar, the track, and who drives.

  ros2 launch falcon_bringup sim.launch.py                    # ADS-DV + Hesai ATX, autonomy
  ros2 launch falcon_bringup sim.launch.py vehicle:=eufs lidar:=ouster_os1_32
  ros2 launch falcon_bringup sim.launch.py mode:=teleop       # drive it yourself
  ros2 launch falcon_bringup sim.launch.py mode:=both         # autonomy; W/A/S/D take over
  ros2 launch falcon_bringup sim.launch.py lidar_z:=0.70 lidar_pitch_deg:=2 coverage:=true
  ros2 launch falcon_bringup sim.launch.py cones:=perception  # drive on YOLO + lidar fusion

Vehicles and lidars are YAML profiles in falcon_description/config/{vehicles,lidars};
`vehicle:=` / `lidar:=` take a profile name or a path to any YAML.  Sensor mounts come
from the vehicle profile; lidar_* / camera_* arguments override them for one run
(x ahead of the wheelbase middle, z above the ground, pitch_deg > 0 = tilted down).

mode       autonomy: map builder + planner + pure pursuit drive the car
           teleop:   keyboard window only
           both:     autonomy drives; any driving key takes over, M hands back
           none:     just the simulator
cones      sim:        drive on the simulator's cones (/cones), no perception needed
           perception: drive on /falcon/fused_cones (YOLO + depth + lidar + fuser)
localization
           slam (default): the stack finds its own pose.  falcon_sim_sensors stands in
                       for the ADS-DV CAN (four wheel rpm, steer angle) and an MTi-8 IMU;
                       the odometry EKF publishes odom -> base_footprint and the cone
                       EKF-SLAM, on lidar_cone_detector_3d's cones, map -> odom.  The
                       simulator's true pose is map -> base_footprint_gt, which only the
                       evaluation reads: localization_eval writes localization.json /
                       .npz after `coverage_laps` laps next to the coverage report, and
                       the coverage counter places cones with the true pose.
           ground_truth: the simulator publishes map -> base_footprint (no lidar needed)
coverage   true: run the cone coverage metric; report after `coverage_laps` laps in
           coverage_output_dir (default ~/falcon_runs).  coverage_source:=raycast (default)
           casts the lidar profile's rays itself from the true pose (exact; the Gazebo
           lidar is not needed, so use_lidar:=false is faster); :=pointcloud counts the
           points of Gazebo's /lidar/points; :=both runs the two side by side (RViz shows
           the ray-cast status low over each cone and Gazebo's above it)

Commands always pass through cmd_mux: pure pursuit -> /falcon/cmd_auto, teleop ->
/falcon/cmd_teleop, mux -> /cmd.
"""

import csv
import json
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, Shutdown, TimerAction)
from launch.launch_description_sources import (AnyLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    ('vehicle', 'ads_dv', 'vehicle profile name or YAML path (ads_dv | eufs)'),
    ('lidar', 'hesai_atx', 'lidar profile name or YAML path'),
    ('track', 'small_track', 'eufs_tracks world, or a track CSV: a path, or a name in runs/tracks or falcon_bringup/tracks'),
    ('mode', 'autonomy', 'autonomy | teleop | both | none'),
    ('cones', 'sim', 'sim (/cones from the simulator) | perception (YOLO + lidar fusion)'),
    ('laps', '0', 'laps for the planner to drive, 0 = unlimited'),
    ('target_speed', '3.0', 'pure pursuit top speed (m/s)'),
    ('use_lidar', 'true', 'spawn the lidar'),
    ('use_camera', 'true', 'spawn the RGB-D camera (needed for cones:=perception)'),
    ('lidar_sensor_type', 'gpu_ray', 'gpu_ray (rendered, fast) | ray (CPU physics, slow)'),
    ('lidar_h_samples', '', 'override columns per frame (thin a dense lidar on a slow PC)'),
    ('lidar_rate_hz', '', 'override lidar frame rate'),
    ('sim_cones_fov_deg', '180', 'view of the simulated /cones (independent of the lidar)'),
    ('ground', 'true', 'visible ground: lidar ground returns (rings), cameras see asphalt'),
    ('localization', 'slam', 'slam (odometry EKF + lidar cone EKF-SLAM) | ground_truth (the simulator\'s pose)'),
    ('wheel_radius_error', '0.01', 'simulated tyre: true rolling radius vs the one the odometry assumes (0.01 = 1 % larger)'),
    ('sensor_seed', '1', 'noise seed of the simulated CAN and IMU (repeat a run with another seed)'),
    ('detector_min_points', '2', 'localization:=slam: fewest lidar points a cluster needs to count '
     'as a cone (it must also stand at least 8 cm tall)'),
    ('coverage', 'false', 'run the cone coverage metric'),
    ('coverage_laps', '1', 'write the coverage report after this many laps'),
    ('coverage_source', 'raycast', 'raycast (exact rays from the true pose) | pointcloud (Gazebo lidar) | both'),
    ('coverage_cone_shape', 'fs', 'fs (rulebook cone sizes) | eufs (the Gazebo cone meshes)'),
    ('coverage_min_points', '3', 'points on a cone to count it detected'),
    ('coverage_min_rings', '1', 'distinct lidar channels on a cone to count it detected'),
    ('coverage_exit_on_finish', 'false', 'end the whole launch when the coverage and localisation reports are written'),
    ('coverage_output_dir', '', 'default $FALCON_RUNS_DIR or ~/falcon_runs'),
    ('coverage_run_name', '', 'default <vehicle>_<lidar>_<time>'),
    ('gazebo_gui', 'true', 'Gazebo window'),
    ('rviz', 'true', 'RViz with the Falcon layout'),
    ('foxglove', 'false', 'foxglove_bridge (with cones:=perception)'),
    ('x', '', 'start pose override (default: car_start in the track CSV)'),
    ('y', '', ''),
    ('yaw', '', ''),
]
MOUNT_ARGS = [f'{s}_{k}' for s in ('lidar', 'camera')
              for k in ('x', 'y', 'z', 'roll_deg', 'pitch_deg', 'yaw_deg')]


# eufs_tracks CSV tags -> cone models.  The ground-truth plugin finds the cones as links
# of the model named `track` and tells their colour from the link name.
CONE_MODELS = {'blue': 'blue_cone', 'yellow': 'yellow_cone', 'orange': 'orange_cone',
               'big_orange': 'big_cone'}


def _resolve_track(track):
    """(name, world file, extra Gazebo model dir or None, track CSV) for track:=.

    An eufs_tracks world by name, or a track CSV in the eufs_tracks format (the track
    editor in falcon_driverless/lidar_coverage_and_hairpins.html exports one): a path
    ending in .csv, or <name>.csv in $FALCON_RUNS_DIR/tracks (~/falcon_runs/tracks, which
    is the repo's runs/tracks in Docker), else in falcon_bringup/tracks (lidar_test_track,
    the lap of docs/lidar_statistics.html).  A CSV becomes a Gazebo world under /tmp at
    every launch, built like the EUFS ones (small_track.world with its cones swapped).
    """
    tracks = get_package_share_directory('eufs_tracks')
    world = os.path.join(tracks, 'worlds', f'{track}.world')
    if not track.endswith('.csv') and os.path.isfile(world):
        return track, world, None, os.path.join(tracks, 'csv', f'{track}.csv')
    runs = os.path.join(os.path.expanduser(os.environ.get('FALCON_RUNS_DIR', '~/falcon_runs')),
                        'tracks')
    path = os.path.expanduser(track if track.endswith('.csv') else f'{track}.csv')
    if not os.path.isfile(path) and not os.path.isabs(path):
        own = os.path.join(get_package_share_directory('falcon_bringup'), 'tracks', path)
        path = own if os.path.isfile(own) and not os.path.isfile(os.path.join(runs, path)) \
            else os.path.join(runs, path)
    if not os.path.isfile(path):
        raise RuntimeError(f'track:={track}: no eufs_tracks world {track}.world, and no {path}')

    name = os.path.splitext(os.path.basename(path))[0]
    with open(path) as f:
        rows = list(csv.DictReader(f))
    includes, count = [], {}
    for row in rows:
        model = CONE_MODELS.get(row.get('tag'))
        if model is None:
            continue
        i = count.get(model, 0)
        count[model] = i + 1
        includes.append(f"    <include>\n      <pose>{float(row['x'])} {float(row['y'])} 0.15 0 0 0"
                        f"</pose>\n      <uri>model://{model}</uri>\n"
                        f"      <name>{model}_{i}</name>\n    </include>\n")
    if not includes:
        raise RuntimeError(f'track:={track}: {path} has no cones (tags: {", ".join(CONE_MODELS)})')

    root = os.path.join('/tmp', f'falcon_track_{os.getpid()}')
    model_dir = os.path.join(root, 'models', 'falcon_imported_track')
    os.makedirs(model_dir, exist_ok=True)
    with open(os.path.join(model_dir, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model>\n  <name>falcon_imported_track</name>\n'
                f'  <version>1.0</version>\n  <sdf version="1.6">model.sdf</sdf>\n'
                f'  <description>{name}, from {path}</description>\n</model>\n')
    with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
        f.write("<?xml version='1.0'?>\n<sdf version='1.6'>\n  <model name='track'>\n"
                + ''.join(includes) + '  </model>\n</sdf>\n')
    with open(os.path.join(tracks, 'worlds', 'small_track.world')) as f:
        template = f.read()
    world = os.path.join(root, f'{name}.world')
    with open(world, 'w') as f:
        f.write(template.replace("<world name='small_track'>", f"<world name='{name}'>")
                .replace('model://small_track', 'model://falcon_imported_track'))
    return name, world, os.path.join(root, 'models'), path


def _start_pose(path):
    try:
        with open(path) as f:
            for row in csv.DictReader(f):
                if row['tag'] == 'car_start':
                    return float(row['x']), float(row['y']), float(row['direction'])
    except (OSError, KeyError, ValueError):
        pass
    return 0.0, 0.0, 0.0


def _setup(context):
    a = {name: LaunchConfiguration(name).perform(context) for name, _, _ in ARGS}
    a.update({name: LaunchConfiguration(name).perform(context) for name in MOUNT_ARGS})
    on = lambda k: a[k].lower() in ('true', '1', 'yes')  # noqa: E731
    mode, cones = a['mode'], a['cones']
    if mode not in ('autonomy', 'teleop', 'both', 'none'):
        raise RuntimeError(f'mode:={mode}: use autonomy, teleop, both or none')
    if cones not in ('sim', 'perception'):
        raise RuntimeError(f'cones:={cones}: use sim or perception')
    if a['localization'] not in ('slam', 'ground_truth'):
        raise RuntimeError(f"localization:={a['localization']}: use slam or ground_truth")
    slam = a['localization'] == 'slam'

    from falcon_description import config as cfg
    from falcon_description.urdf_builder import build, lidar_topics
    overrides = {k: a[k] for k in MOUNT_ARGS}
    overrides.update(lidar_h_samples=a['lidar_h_samples'], lidar_rate_hz=a['lidar_rate_hz'])
    vehicle = cfg.load_vehicle(a['vehicle'], overrides)
    lidar = cfg.load_lidar(a['lidar'], overrides)
    use_lidar, use_camera = on('use_lidar'), on('use_camera')
    if cones == 'perception' and not (use_lidar and use_camera):
        raise RuntimeError('cones:=perception needs use_lidar:=true and use_camera:=true')
    if slam and not use_lidar:
        raise RuntimeError('localization:=slam needs the lidar (use_lidar:=true), or use '
                           'localization:=ground_truth')
    truth_frame = 'base_footprint_gt' if slam else 'base_footprint'
    teleop = mode in ('teleop', 'both')
    autonomy = mode in ('autonomy', 'both')

    urdf = build(vehicle, lidar, simulate_perception=True, lidar_enabled=use_lidar,
                 camera_enabled=use_camera, teleop_camera=teleop,
                 sensor_type=a['lidar_sensor_type'],
                 sim_cone_fov_deg=float(a['sim_cones_fov_deg']), truth_frame=truth_frame)
    urdf_path = os.path.join('/tmp', f'falcon_{os.getpid()}.urdf')
    with open(urdf_path, 'w') as f:
        f.write(urdf)

    track_name, world, track_models, track_csv = _resolve_track(a['track'])

    x0, y0, yaw0 = _start_pose(track_csv)
    x0 = float(a['x']) if a['x'] else x0
    y0 = float(a['y']) if a['y'] else y0
    yaw0 = float(a['yaw']) if a['yaw'] else yaw0

    tracks = get_package_share_directory('eufs_tracks')
    plugin_lib = os.path.join(get_package_prefix('eufs_plugins'), 'lib')
    ros_lib = os.path.join('/opt/ros', os.environ.get('ROS_DISTRO', 'humble'), 'lib')
    resources = ':'.join([
        os.path.join(get_package_share_directory('eufs_sensors'), 'meshes'),
        os.path.join(tracks, 'meshes'), os.path.join(tracks, 'materials'),
        os.path.join(get_package_share_directory('eufs_racecar'), 'meshes'),
        os.path.join(get_package_share_directory('eufs_racecar'), 'materials'),
        '/usr/share/gazebo-11'])
    sim_time = {'use_sim_time': True}
    s = vehicle['sensors']
    lm = s['lidar']
    box = vehicle['body_box']
    body_box = [box['x_min'], box['x_max'], box['y_half'], box['z_max']]
    cones_topic = '/cones' if cones == 'sim' else '/falcon/fused_cones'

    actions = [
        LogInfo(msg=f"[falcon sim] {vehicle['description']} on {track_name}"
                    f"{'' if track_models is None else f' (from {track_csv})'}, mode={mode}, "
                    f"cones={cones}, localization={a['localization']}"),
        LogInfo(msg=f"[falcon sim] lidar: {cfg.lidar_summary(lidar) if use_lidar else 'off'}"),
        LogInfo(msg=f"[falcon sim] lidar mount x={lm['x']:.3f} z={lm['z']:.3f} m, pitch "
                    f"{lm['pitch_deg']:+.1f} deg; camera z={s['camera']['z']:.3f} m pitch "
                    f"{s['camera']['pitch_deg']:+.1f} deg; URDF {urdf_path}"),
        # Big shared-memory segments, so dense lidar clouds arrive whole (see the file).
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', os.path.join(
            get_package_share_directory('falcon_bringup'), 'config', 'fastdds_sim.xml')),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', ':'.join(
            p for p in (track_models, os.path.join(tracks, 'models')) if p)),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', f'{ros_lib}:{plugin_lib}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world,
                              'gui': a['gazebo_gui'], 'verbose': 'false',
                              'pause': 'false'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_robot',
             output='screen',
             arguments=['-entity', 'eufs', '-file', urdf_path, '-x', str(x0), '-y', str(y0),
                        '-z', '0.1', '-Y', str(yaw0), '-timeout', '600',
                        '--ros-args', '--log-level', 'warn']),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, 'rate': 50, **sim_time}],
             remappings=[('/joint_states', '/eufs/joint_states')]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': urdf, **sim_time}],
             remappings=[('/joint_states', '/eufs/joint_states')],
             arguments=['--ros-args', '--log-level', 'warn']),
        # EUFS ignores /cmd until a mission is set; this sets manual drive and exits.
        TimerAction(period=3.0, actions=[
            Node(package='falcon_bringup', executable='enable_manual_drive',
                 name='enable_manual_drive', output='screen')]),
        Node(package='falcon_teleop', executable='cmd_mux', name='cmd_mux', output='screen'),
    ]

    if on('ground'):
        # Visual only: EUFS's ground_plane is collision only, which gpu_ray and the
        # cameras cannot see.  The cone detector and the coverage counter drop points
        # within a few cm of z = 0, so the ground needs no further segmentation.
        actions.append(Node(
            package='gazebo_ros', executable='spawn_entity.py', name='spawn_ground',
            output='screen',
            arguments=['-entity', 'falcon_ground', '-file',
                       os.path.join(get_package_share_directory('falcon_bringup'), 'config',
                                    'ground_visual.sdf'),
                       '-timeout', '600', '--ros-args', '--log-level', 'warn']))

    if use_lidar and len(lidar['groups']) > 1:
        actions.append(Node(package='falcon_cone_perception', executable='lidar_merger.py',
                            name='lidar_merger', output='screen',
                            parameters=[{'input_topics': lidar_topics(lidar), **sim_time}]))

    if cones == 'perception':
        actions.append(TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('falcon_cone_perception'), 'launch',
                'perception.launch.py')),
            launch_arguments={'lidar_detector': '3d',
                              'body_box': ','.join(str(v) for v in body_box),
                              'foxglove': a['foxglove']}.items())]))

    if autonomy:
        planning = get_package_share_directory('falcon_planning')
        actions += [
            Node(package='falcon_cone_map_builder', executable='falcon_cone_map_builder_node',
                 name='falcon_cone_map_builder_node', output='screen',
                 parameters=[os.path.join(get_package_share_directory('falcon_cone_map_builder'),
                                          'config', 'cone_map_builder.yaml'),
                             {'input_topic': cones_topic, **sim_time}]),
            Node(package='falcon_planning', executable='path_planner_node',
                 name='path_planner_node', output='screen',
                 parameters=[os.path.join(planning, 'config', 'path_planner.yaml'),
                             {'cones_topic': cones_topic, 'total_laps': int(a['laps']),
                              **sim_time}]),
            Node(package='falcon_planning', executable='pure_pursuit_node',
                 name='pure_pursuit_node', output='screen',
                 parameters=[os.path.join(planning, 'config', 'pure_pursuit.yaml'),
                             {'cmd_topic': '/falcon/cmd_auto',
                              'wheelbase': float(vehicle['wheelbase_m']),
                              'max_steering_angle': float(vehicle['max_steering_rad']),
                              'target_speed': float(a['target_speed']), **sim_time}]),
        ]

    # numpy's OpenBLAS starts a thread per core in every process; several numpy nodes
    # next to Gazebo then starve each other.  One thread each, four for the SLAM's
    # covariance update (7.5 ms a scan for 430 cones on four threads, 16.5 ms on one).
    blas1 = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}
    if slam:
        wheel = {'wheelbase': float(vehicle['wheelbase_m']), 'track': float(vehicle['track_m']),
                 'wheel_radius': float(vehicle['wheel_diameter_m']) / 2, **sim_time}
        actions += [
            Node(package='falcon_sim_sensors', executable='vehicle_sensors_sim',
                 name='vehicle_sensors_sim', output='screen', additional_env=blas1,
                 parameters=[{**wheel, 'rolling_radius_error': float(a['wheel_radius_error']),
                              'seed': int(a['sensor_seed'])}]),
            Node(package='falcon_localization', executable='odometry_ekf', name='odometry_ekf',
                 output='screen', additional_env=blas1, parameters=[wheel]),
            Node(package='falcon_localization', executable='cone_slam', name='cone_slam',
                 output='screen',
                 additional_env={'OPENBLAS_NUM_THREADS': '4', 'OMP_NUM_THREADS': '4'},
                 parameters=[{'initial_pose': [x0, y0, yaw0],
                              'sensor_xy': [float(lm['x']), float(lm['y'])], **sim_time}]),
        ]
        if cones != 'perception':
            # The SLAM's own lidar cone detector (cones:=perception already runs one).
            # Range: the lidar's datasheet range, so no lidar is cut short.
            actions.append(Node(
                package='falcon_cone_perception', executable='lidar_cone_detector_3d.py',
                name='lidar_cone_detector', output='screen', additional_env=blas1,
                parameters=[{'body_box': body_box,
                             'max_range_m': float(lidar['range']['max_m']),
                             'min_points': int(a['detector_min_points']), **sim_time}]))

    if teleop:
        actions.append(Node(
            package='falcon_teleop', executable='keyboard_teleop', name='keyboard_teleop',
            output='screen',
            parameters=[{'cmd_topic': '/falcon/cmd_teleop', 'start_in_auto': autonomy,
                         'max_steering': float(vehicle['max_steering_rad'])}]))

    if a['coverage_source'] not in ('raycast', 'pointcloud', 'both'):
        raise RuntimeError(f"coverage_source:={a['coverage_source']}: use raycast, pointcloud or both")
    if on('coverage') and a['coverage_source'] != 'raycast' and not use_lidar:
        raise RuntimeError(f"coverage_source:={a['coverage_source']} needs use_lidar:=true")
    report_nodes = []          # (name, kwargs) of nodes that end the run when done
    if on('coverage') or slam:
        run_name = a['coverage_run_name'] or None
        info = {
            'vehicle': {k: vehicle[k] for k in ('name', 'description', 'wheelbase_m',
                                                 'max_steering_rad')},
            'lidar': {'name': lidar.get('name'), 'file': lidar['_file'],
                      'channels': len(lidar['channels_deg']),
                      'channels_deg': lidar['channels_deg'],
                      'horizontal': lidar['horizontal'], 'rate_hz': lidar['rate_hz'],
                      'range_max_m': float(lidar['range']['max_m'])},
            'lidar_mount': s['lidar'],
            'track': track_name, 'mode': mode, 'cones': cones,
            'target_speed': float(a['target_speed']),
            'localization': a['localization'],
            'wheel_radius_error': float(a['wheel_radius_error']) if slam else None,
            'sensor_seed': int(a['sensor_seed']) if slam else None,
            'detector_min_points': int(a['detector_min_points']) if slam else None,
        }
        if not run_name:
            import datetime
            run_name = (f"{vehicle['name']}_{os.path.splitext(os.path.basename(lidar['_file']))[0]}"
                        f"_{datetime.datetime.now():%Y%m%d_%H%M%S}")
        out_dir = {'output_dir': a['coverage_output_dir']} if a['coverage_output_dir'] else {}
    if slam:
        report_nodes.append(('localization_eval', dict(
            package='falcon_evaluation', executable='localization_eval',
            parameters=[{'laps': int(a['coverage_laps']), 'run_name': run_name,
                         'exit_on_finish': on('coverage_exit_on_finish'),
                         'run_info_json': json.dumps(info), **out_dir, **sim_time}])))
    if on('coverage'):
        params = {
            'channels_deg': [float(v) for v in lidar['channels_deg']],
            'hfov_deg': float(lidar['horizontal']['fov_deg']),
            'hcentre_deg': float(lidar['horizontal'].get('centre_deg', 0.0)),
            'max_range_m': float(lidar['range']['max_m']),
            'min_points': int(a['coverage_min_points']),
            'min_rings': int(a['coverage_min_rings']),
            'laps': int(a['coverage_laps']),
            'body_box': [float(v) for v in body_box],
            'source': a['coverage_source'],
            'cone_shape': a['coverage_cone_shape'],
            'rate_hz': float(lidar['rate_hz']),
            'h_samples': int(lidar['horizontal']['samples']),
            'min_range_m': float(lidar['range']['min_m']),
            'lidar_mount': [float(lm[k]) for k in ('x', 'y', 'z', 'roll_deg', 'pitch_deg',
                                                   'yaw_deg')],
            'occluder_box': [float(v) for v in vehicle.get('occluder_box', {}).values()]
            if vehicle.get('occluder_box') else [0.0],
            'exit_on_finish': on('coverage_exit_on_finish'),
            'run_info_json': json.dumps(info),
            'pose_frame': truth_frame,
            **out_dir,
            **sim_time,
        }
        # both: the ray-cast and Gazebo's own cloud side by side on the same drive,
        # reports in <run>/raycast and <run>/pointcloud; the Gazebo one publishes
        # under /coverage/pointcloud/.
        if a['coverage_source'] == 'both':
            nodes = [('cone_coverage', 'raycast', f'{run_name}/raycast', [], 0.6),
                     ('cone_coverage_pointcloud', 'pointcloud', f'{run_name}/pointcloud',
                      [(f'/coverage/{t}', f'/coverage/pointcloud/{t}')
                       for t in ('markers', 'summary', 'write_report')], 1.4)]
        else:
            nodes = [('cone_coverage', a['coverage_source'], run_name, [], 0.6)]
        for name, source, run, remaps, marker_z in nodes:
            report_nodes.append((name, dict(
                package='falcon_evaluation', executable='cone_coverage',
                parameters=[{**params, 'source': source, 'run_name': run,
                             'marker_z_m': marker_z}],
                remappings=remaps)))

    exited = []

    def _report_exit(event, context):
        # End the launch once every report node (coverage, localisation) is done.
        exited.append(event.action.name)
        if len(exited) == len(report_nodes):
            return [Shutdown(reason='reports written')]
        return None

    for name, kw in report_nodes:
        actions.append(Node(name=name, output='screen', additional_env=blas1, **kw,
                            on_exit=_report_exit if on('coverage_exit_on_finish') else None))

    if on('rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz', output='log',
            arguments=['-d', os.path.join(get_package_share_directory('falcon_bringup'),
                                          'config', 'falcon.rviz')],
            parameters=[sim_time]))
    if on('foxglove') and cones != 'perception':
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory('foxglove_bridge'), 'launch',
            'foxglove_bridge_launch.xml'))))
    return actions


def generate_launch_description():
    decl = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
    decl += [DeclareLaunchArgument(n, default_value='', description='mount override')
             for n in MOUNT_ARGS]
    return LaunchDescription(decl + [OpaqueFunction(function=_setup)])
