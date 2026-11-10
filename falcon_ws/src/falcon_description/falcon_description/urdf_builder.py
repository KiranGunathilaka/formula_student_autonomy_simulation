"""
Build the Falcon car URDF from a vehicle profile and a lidar profile.

The chassis, wheels, camera and EUFS plugins come from urdf/falcon_car.urdf.xacro.
The lidar is generated here because its shape depends on the profile: one Gazebo ray
sensor per evenly spaced run of channels (see config.uniform_groups).  A single run
publishes straight to ``points_topic``; several runs publish to
``<points_topic>_raw/g<i>`` and falcon_cone_perception's lidar_merger joins them.
"""

import math
import os
from xml.dom import minidom

import xacro
from ament_index_python.packages import get_package_share_directory


CHASSIS_Z = 0.505 / 2.0   # eufs_base.urdf.xacro: chassis link above base_footprint
LIDAR_FRAME = 'lidar_link'


def _xyz(p, dz=-CHASSIS_Z):
    return f"{p['x']:.4f} {p['y']:.4f} {p['z'] + dz:.4f}"


def _rpy(p):
    return (f"{math.radians(p['roll_deg']):.5f} {math.radians(p['pitch_deg']):.5f} "
            f"{math.radians(p['yaw_deg']):.5f}")


def lidar_topics(lidar, points_topic='/lidar/points'):
    """Topics the Gazebo ray sensors publish on, one per channel run."""
    n = len(lidar['groups'])
    if n == 1:
        return [points_topic]
    return [f'{points_topic}_raw/g{i}' for i in range(n)]


def _lidar_xml(lidar, mount, sensor_type, points_topic):
    h = lidar['horizontal']
    fov = math.radians(float(h['fov_deg']))
    n_h = int(h['samples'])
    centre = math.radians(float(h.get('centre_deg', 0.0)))
    if fov >= 2 * math.pi - 1e-6:
        # Gazebo includes both ends; drop the duplicate column at +-180 deg.
        h_min, h_max = -math.pi, math.pi - 2 * math.pi / n_h
    else:
        h_min, h_max = centre - fov / 2, centre + fov / 2
    rng = lidar['range']
    rate = float(lidar['rate_hz'])
    noise = float(lidar.get('noise_stddev_m', 0.02))
    spinning = lidar.get('kind', 'spinning') == 'spinning'
    geom = ('<cylinder radius="0.05" length="0.07"/>' if spinning
            else '<box size="0.10 0.12 0.05"/>')
    topics = lidar_topics(lidar, points_topic)

    parts = [f'''
  <link name="{LIDAR_FRAME}">
    <visual><geometry>{geom}</geometry>
      <material name="lidar_black"><color rgba="0.1 0.1 0.1 1.0"/></material></visual>
    <inertial><mass value="0.5"/><origin xyz="0 0 0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/></inertial>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="chassis"/>
    <child link="{LIDAR_FRAME}"/>
    <origin xyz="{_xyz(mount)}" rpy="{_rpy(mount)}"/>
  </joint>''']
    for i, ((lo, hi, count), topic) in enumerate(zip(lidar['groups'], topics)):
        lo_r, hi_r = math.radians(lo), math.radians(hi)
        parts.append(f'''
  <gazebo reference="{LIDAR_FRAME}">
    <sensor type="{sensor_type}" name="lidar_g{i}">
      <always_on>true</always_on>
      <visualize>false</visualize>
      <update_rate>{rate}</update_rate>
      <pose>0 0 0 0 0 0</pose>
      <ray>
        <scan>
          <horizontal><samples>{n_h}</samples><resolution>1</resolution>
            <min_angle>{h_min:.6f}</min_angle><max_angle>{h_max:.6f}</max_angle></horizontal>
          <vertical><samples>{count}</samples><resolution>1</resolution>
            <min_angle>{lo_r:.6f}</min_angle><max_angle>{hi_r:.6f}</max_angle></vertical>
        </scan>
        <range><min>{float(rng["min_m"])}</min><max>{float(rng["max_m"])}</max>
          <resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>{noise}</stddev></noise>
      </ray>
      <plugin name="lidar_g{i}_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros><remapping>~/out:={topic}</remapping></ros>
        <output_type>sensor_msgs/PointCloud2</output_type>
        <frame_name>{LIDAR_FRAME}</frame_name>
      </plugin>
    </sensor>
  </gazebo>''')
    return '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">' + ''.join(parts) + '</robot>'


def build(vehicle, lidar, *, simulate_perception=True, lidar_enabled=True,
          camera_enabled=True, teleop_camera=False, sensor_type='gpu_ray',
          vehicle_model='DynamicBicycle', command_mode='velocity', publish_tf=True,
          sim_cone_range=20.0, sim_cone_fov_deg=180.0, points_topic='/lidar/points',
          truth_frame='base_footprint'):
    """Return the URDF as a string.

    vehicle, lidar: dicts from config.load_vehicle / config.load_lidar.
    simulate_perception: the EUFS plugin also publishes simulated cones on /cones.
    sim_cone_fov_deg / sim_cone_range: the view of those simulated cones.  Kept
    independent of the lidar under test so that runs with different lidars drive the
    same way when they drive on /cones.
    truth_frame: child frame of the simulator's true-pose TF (map -> truth_frame).
    base_footprint when the stack uses the true pose; another name (base_footprint_gt)
    when the stack's localisation publishes map -> odom -> base_footprint itself.
    """
    racecar = get_package_share_directory('eufs_racecar')
    models = get_package_share_directory('eufs_models')
    plugins = get_package_share_directory('eufs_plugins')
    here = get_package_share_directory('falcon_description')
    s = vehicle['sensors']
    cam = vehicle.get('camera', {})

    mappings = {
        'config_file': os.path.join(racecar, 'robots', vehicle['eufs_robot'], 'configDry.yaml'),
        'noise_config': os.path.join(models, 'config', 'noise.yaml'),
        'recolor_config': os.path.join(plugins, 'config', 'cone_recolor.yaml'),
        'vehicle_model': vehicle_model,
        'command_mode': command_mode,
        'publish_tf': str(publish_tf).lower(),
        'truth_frame': truth_frame,
        'pub_ground_truth': 'true',
        'simulate_perception': str(simulate_perception).lower(),
        'chassis_mesh': vehicle['chassis_mesh'],
        'wheelbase': str(vehicle['wheelbase_m']),
        'track': str(vehicle['track_m']),
        'sim_lidar_fov': f'{math.radians(sim_cone_fov_deg):.4f}',
        'sim_lidar_range': str(sim_cone_range),
        'sim_camera_fov': f"{math.radians(float(cam.get('hfov_deg', 110))):.4f}",
        'sim_camera_range': '15.0',
        'camera_enabled': str(camera_enabled).lower(),
        'camera_xyz': _xyz(s['camera']),
        'camera_rpy': _rpy(s['camera']),
        'camera_hfov': f"{math.radians(float(cam.get('hfov_deg', 110))):.4f}",
        'camera_width': str(cam.get('width', 1280)),
        'camera_height': str(cam.get('height', 720)),
        'camera_rate': str(cam.get('rate_hz', 15)),
        'teleop_camera_enabled': str(teleop_camera).lower(),
        'teleop_camera_xyz': _xyz(s['teleop_camera']),
        'teleop_camera_rpy': _rpy(s['teleop_camera']),
        'imu_xyz': _xyz(s['imu']),
        'gps_xyz': _xyz(s['gps']),
    }
    doc = xacro.process_file(os.path.join(here, 'urdf', 'falcon_car.urdf.xacro'),
                             mappings=mappings)
    if lidar_enabled:
        snippet = minidom.parseString(
            _lidar_xml(lidar, s['lidar'], sensor_type, points_topic))
        robot = doc.documentElement
        for node in list(snippet.documentElement.childNodes):
            if node.nodeType == node.ELEMENT_NODE:
                robot.appendChild(doc.importNode(node, True))
    return doc.toprettyxml(indent='  ')
