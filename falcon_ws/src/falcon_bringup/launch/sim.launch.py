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
    ('track', 'small_track', 'eufs_tracks world'),
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
    ('coverage', 'false', 'run the cone coverage metric'),
    ('coverage_laps', '1', 'write the coverage report after this many laps'),
    ('coverage_source', 'raycast', 'raycast (exact rays from the true pose) | pointcloud (Gazebo lidar) | both'),
    ('coverage_cone_shape', 'fs', 'fs (rulebook cone sizes) | eufs (the Gazebo cone meshes)'),
    ('coverage_min_points', '3', 'points on a cone to count it detected'),
    ('coverage_min_rings', '1', 'distinct lidar channels on a cone to count it detected'),
    ('coverage_exit_on_finish', 'false', 'end the whole launch when the report is written'),
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


def _start_pose(track):
    path = os.path.join(get_package_share_directory('eufs_tracks'), 'csv', f'{track}.csv')
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

    from falcon_description import config as cfg
    from falcon_description.urdf_builder import build, lidar_topics
    overrides = {k: a[k] for k in MOUNT_ARGS}
    overrides.update(lidar_h_samples=a['lidar_h_samples'], lidar_rate_hz=a['lidar_rate_hz'])
    vehicle = cfg.load_vehicle(a['vehicle'], overrides)
    lidar = cfg.load_lidar(a['lidar'], overrides)
    use_lidar, use_camera = on('use_lidar'), on('use_camera')
    if cones == 'perception' and not (use_lidar and use_camera):
        raise RuntimeError('cones:=perception needs use_lidar:=true and use_camera:=true')
    teleop = mode in ('teleop', 'both')
    autonomy = mode in ('autonomy', 'both')

    urdf = build(vehicle, lidar, simulate_perception=True, lidar_enabled=use_lidar,
                 camera_enabled=use_camera, teleop_camera=teleop,
                 sensor_type=a['lidar_sensor_type'],
                 sim_cone_fov_deg=float(a['sim_cones_fov_deg']))
    urdf_path = os.path.join('/tmp', f'falcon_{os.getpid()}.urdf')
    with open(urdf_path, 'w') as f:
        f.write(urdf)

    x0, y0, yaw0 = _start_pose(a['track'])
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
        LogInfo(msg=f"[falcon sim] {vehicle['description']} on {a['track']}, mode={mode}, "
                    f"cones={cones}"),
        LogInfo(msg=f"[falcon sim] lidar: {cfg.lidar_summary(lidar) if use_lidar else 'off'}"),
        LogInfo(msg=f"[falcon sim] lidar mount x={lm['x']:.3f} z={lm['z']:.3f} m, pitch "
                    f"{lm['pitch_deg']:+.1f} deg; camera z={s['camera']['z']:.3f} m pitch "
                    f"{s['camera']['pitch_deg']:+.1f} deg; URDF {urdf_path}"),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(tracks, 'models')),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', f'{ros_lib}:{plugin_lib}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': os.path.join(tracks, 'worlds', f"{a['track']}.world"),
                              'gui': a['gazebo_gui'], 'verbose': 'false',
                              'pause': 'false'}.items()),
        Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_robot',
             output='screen',
             arguments=['-entity', 'eufs', '-file', urdf_path, '-x', str(x0), '-y', str(y0),
                        '-z', '0.1', '-Y', str(yaw0), '-spawn_service_timeout', '60.0',
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
    if on('coverage'):
        run_name = a['coverage_run_name'] or None
        info = {
            'vehicle': {k: vehicle[k] for k in ('name', 'description', 'wheelbase_m',
                                                 'max_steering_rad')},
            'lidar': {'name': lidar.get('name'), 'file': lidar['_file'],
                      'channels': len(lidar['channels_deg']),
                      'channels_deg': lidar['channels_deg'],
                      'horizontal': lidar['horizontal'], 'rate_hz': lidar['rate_hz']},
            'lidar_mount': s['lidar'],
            'track': a['track'], 'mode': mode, 'cones': cones,
            'target_speed': float(a['target_speed']),
        }
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
            **sim_time,
        }
        if a['coverage_output_dir']:
            params['output_dir'] = a['coverage_output_dir']
        if not run_name:
            import datetime
            run_name = (f"{vehicle['name']}_{os.path.splitext(os.path.basename(lidar['_file']))[0]}"
                        f"_{datetime.datetime.now():%Y%m%d_%H%M%S}")
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
        exited = []

        def _coverage_exit(event, context):
            # End the launch once every coverage node has written its report.
            exited.append(event.action.name)
            if len(exited) == len(nodes):
                return [Shutdown(reason='coverage report written')]
            return None

        for name, source, run, remaps, marker_z in nodes:
            actions.append(Node(
                package='falcon_evaluation', executable='cone_coverage', name=name,
                output='screen', parameters=[{**params, 'source': source, 'run_name': run,
                                              'marker_z_m': marker_z}],
                remappings=remaps,
                on_exit=_coverage_exit if on('coverage_exit_on_finish') else None))

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
