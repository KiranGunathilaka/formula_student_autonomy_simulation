#!/usr/bin/env python3
"""
Keyboard teleop for EUFS simulation, with a chase-camera view.

Controls (click the window first so it has keyboard focus):
  W / Up      accelerate towards the cruise speed      S / Down   brake
  A / Left    steer left                               D / Right  steer right
  C           cruise on/off: hold the cruise speed without holding W
  R / F       cruise speed +/- `speed_step`
  Space       stop (and take manual control)
  M           MANUAL <-> AUTO (AUTO: stop publishing, the autonomy stack drives)

Driving keys take over immediately when in AUTO.  With cmd_topic:=auto (the
default) the node publishes to /falcon/cmd_teleop once it hears cmd_mux on
/falcon/control_source (falcon_bringup sim.launch.py always runs it), so the mux
gives it priority over the planner; with no mux it publishes straight to /cmd.  If
the mux reports the planner driving when the window opens, it starts in AUTO.

Speed ramps (accel / coast / brake) rather than jumping, and steering moves at
`steering_rate` rad/s, so short taps make small corrections.  X11 key auto-repeat
(a release + press pair every ~30 ms while a key is held) is filtered out.

Uses tkinter (no PyQt) to avoid Qt/cv2 plugin conflicts.
"""
import sys
import threading
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import CanState
from eufs_msgs.srv import SetCanState
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

try:
    import tkinter as tk
    from tkinter import font as tkfont
    HAS_TK = True
except ImportError:
    HAS_TK = False

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV2 = True
except ImportError:
    CvBridge = None
    cv2 = None
    HAS_CV2 = False

try:
    import PIL.Image
    import PIL.ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

HAS_IMAGE = CvBridge is not None and HAS_PIL and HAS_CV2

# The chase camera exists when sim.launch.py is started with teleop; the ZED view
# is the fallback (and the older EUFS URDFs' own teleop camera).
IMAGE_TOPIC_FALLBACKS = [
    '/teleop_camera/image_raw',
    '/zed/image_raw',
    '/zed/left/image_rect_color',
]

KEYS = {
    'w': 'forward', 'up': 'forward',
    's': 'brake', 'down': 'brake',
    'a': 'left', 'left': 'left',
    'd': 'right', 'right': 'right',
}
RELEASE_DEBOUNCE_S = 0.06


class TeleopWindow:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Falcon Teleop')
        self.root.geometry('900x640')
        self.root.configure(bg='#1e1e1e')

        self.bridge = CvBridge() if HAS_IMAGE else None
        self.last_image = None
        self.photo = None
        self._img_pending = False

        bfont = tkfont.Font(size=11, weight='bold')
        tk.Label(self.root, fg='#eee', bg='#1e1e1e', font=bfont,
                 text='W/S: drive/brake   A/D: steer   C: cruise   R/F: cruise speed   '
                      'Space: stop   M: manual/auto').pack(pady=4)

        self.mode_var = tk.StringVar(value='')
        self.mode_lbl = tk.Label(self.root, textvariable=self.mode_var,
                                 font=tkfont.Font(size=14, weight='bold'), bg='#1e1e1e')
        self.mode_lbl.pack(pady=2)
        self.status_var = tk.StringVar(value='')
        tk.Label(self.root, textvariable=self.status_var, fg='#bbb', bg='#1e1e1e',
                 font=('', 10)).pack(pady=1)
        self.info_var = tk.StringVar(value='')
        tk.Label(self.root, textvariable=self.info_var, fg='#8fbc8f', bg='#1e1e1e',
                 font=('', 9), wraplength=860, justify=tk.LEFT).pack(pady=1)

        self.img_frame = tk.Frame(self.root, bg='#2a2a2a', width=800, height=450)
        self.img_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)
        self.img_frame.pack_propagate(False)
        self.img_canvas = tk.Canvas(self.img_frame, bg='#2a2a2a', highlightthickness=0,
                                    width=800, height=450)
        self.img_canvas.pack(fill=tk.BOTH, expand=True)
        self.img_canvas.create_text(
            400, 225, fill='#888', font=('', 11), tags='placeholder',
            text='Waiting for camera...\nClick this window to enable keyboard controls')
        self._canvas_image_id = None

        self._held = set()
        self._release_after = {}
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self.root.bind_all('<KeyRelease>', self._on_key_release)

        def _take_focus(e=None):
            self.root.focus_force()
        self.root.after(100, _take_focus)
        self.img_canvas.bind('<Button-1>', _take_focus)
        self.root.after(100, self._tick_ui)

    # ---------------------------------------------------------------- keys
    def _on_key_press(self, e):
        k = (e.keysym or '').lower()
        if k in KEYS:
            pending = self._release_after.pop(k, None)
            if pending is not None:
                self.root.after_cancel(pending)   # auto-repeat: the key is still held
            if k not in self._held:
                self._held.add(k)
                self.node.set_key(KEYS[k], True)
            return
        if k == 'space':
            self.node.stop()
        elif k == 'c':
            self.node.toggle_cruise()
        elif k == 'r':
            self.node.change_cruise(+1)
        elif k == 'f':
            self.node.change_cruise(-1)
        elif k == 'm':
            self.node.toggle_mode()

    def _on_key_release(self, e):
        k = (e.keysym or '').lower()
        if k in KEYS and k in self._held:
            self._release_after[k] = self.root.after(
                int(RELEASE_DEBOUNCE_S * 1000), lambda k=k: self._really_released(k))

    def _really_released(self, k):
        self._release_after.pop(k, None)
        self._held.discard(k)
        if not any(KEYS[h] == KEYS[k] for h in self._held):
            self.node.set_key(KEYS[k], False)

    # ---------------------------------------------------------------- image
    def image_callback(self, msg):
        if not HAS_IMAGE or self.bridge is None:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if img is None or img.size == 0:
                return
            if len(img.shape) == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            elif img.shape[2] == 3 and (msg.encoding or 'bgr8').lower() != 'rgb8':
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.last_image = np.ascontiguousarray(img)
            if not self._img_pending:
                self._img_pending = True
                self.root.after(0, self._update_image)
        except Exception as e:
            self.node.get_logger().warn(f'Image conversion failed: {e}',
                                        throttle_duration_sec=5.0)

    def _update_image(self):
        self._img_pending = False
        if self.last_image is None or not self.root.winfo_exists():
            return
        try:
            self.img_canvas.delete('placeholder')
            pil_img = PIL.Image.fromarray(self.last_image)
            cw, ch = self.img_canvas.winfo_width(), self.img_canvas.winfo_height()
            if cw < 10 or ch < 10:
                cw, ch = 800, 450
            pil_img.thumbnail((cw, ch))
            self.photo = PIL.ImageTk.PhotoImage(pil_img)
            if self._canvas_image_id is not None:
                self.img_canvas.delete(self._canvas_image_id)
            self._canvas_image_id = self.img_canvas.create_image(
                cw // 2, ch // 2, image=self.photo, anchor=tk.CENTER)
        except Exception as e:
            self.node.get_logger().warn(f'Image display failed: {e}', throttle_duration_sec=5.0)

    # ---------------------------------------------------------------- status
    def _tick_ui(self):
        if not self.root.winfo_exists():
            return
        n = self.node
        manual = n.mode == 'MANUAL'
        self.mode_var.set(f"{n.mode}{'  |  CRUISE' if n.cruise and manual else ''}"
                          f"   (driving: {n.control_source})")
        self.mode_lbl.configure(fg='#ffb347' if manual else '#7ec8e3')
        self.status_var.set(
            f'speed {n.speed:4.1f} m/s   cruise {n.cruise_speed:.1f} m/s   '
            f'steer {n.steering:+.2f} rad   mission: {n.mission_text}')
        extra = f'lap {n.lap_count}' if n.lap_count is not None else ''
        if n.coverage_text:
            extra = (extra + '   |   ' if extra else '') + n.coverage_text
        self.info_var.set(extra)
        self.root.after(100, self._tick_ui)

    def run(self):
        self.root.mainloop()


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        p = self.declare_parameter
        p('cmd_topic', 'auto')     # auto: /falcon/cmd_teleop behind cmd_mux, else /cmd
        p('image_topic', '/teleop_camera/image_raw')
        p('publish_rate', 50.0)
        p('cruise_speed', 3.0)
        p('speed_step', 0.5)
        p('max_speed', 10.0)
        p('accel', 2.0)            # m/s^2 while W is held
        p('coast_decel', 0.8)      # m/s^2 with no key
        p('brake_decel', 5.0)      # m/s^2 while S is held
        p('max_steering', 0.3665)  # rad; ADS-DV AI limit (sim.launch.py passes the vehicle's)
        p('steering_rate', 0.8)    # rad/s while A/D is held
        p('steering_return_rate', 1.2)
        p('start_in_auto', False)  # also switches to AUTO by itself if the mux reports 'auto'
        p('manage_mission', True)  # put EUFS into manual-drive mission so /cmd is accepted

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.cruise_speed = float(g('cruise_speed'))
        self.speed_step = float(g('speed_step'))
        self.max_speed = float(g('max_speed'))
        self.accel = float(g('accel'))
        self.coast = float(g('coast_decel'))
        self.brake_decel = float(g('brake_decel'))
        self.max_steering = float(g('max_steering'))
        self.steer_rate = float(g('steering_rate'))
        self.steer_return = float(g('steering_return_rate'))

        self.mode = 'AUTO' if g('start_in_auto') else 'MANUAL'
        self.cruise = False
        self.keys = {'forward': False, 'brake': False, 'left': False, 'right': False}
        self.speed = 0.0
        self.steering = 0.0
        self.control_source = '?'
        self.lap_count = None
        self.coverage_text = ''
        self.mission_text = 'unknown'
        self._ami_manual = False
        self._last_t = time.monotonic()
        self._t_start = self._last_t
        self.window = None

        cmd_topic = g('cmd_topic')
        self._detect_mux = cmd_topic == 'auto'
        self._mux_seen = False
        self._first_key = False
        self.pub = self.create_publisher(
            AckermannDriveStamped, '/cmd' if self._detect_mux else cmd_topic, 10)
        self.pub_mux = (self.create_publisher(AckermannDriveStamped, '/falcon/cmd_teleop', 10)
                        if self._detect_mux else None)
        image_topic = g('image_topic')
        self.create_subscription(Image, image_topic, self._img_cb, 10)
        for t in IMAGE_TOPIC_FALLBACKS:
            if t != image_topic:
                self.create_subscription(Image, t, lambda m, t=t: self._img_cb(m, t), 10)
        self._img_topic = None
        self.create_subscription(String, '/falcon/control_source', self._source_cb, 10)
        self.create_subscription(Int32, '/planning/lap_count',
                                 lambda m: setattr(self, 'lap_count', m.data), 10)
        self.create_subscription(String, '/coverage/summary',
                                 lambda m: setattr(self, 'coverage_text', m.data), 10)
        self.create_subscription(CanState, '/ros_can/state', self._can_state_cb, 10)

        self.create_timer(1.0 / float(g('publish_rate')), self.publish_cmd)
        if g('manage_mission'):
            self._mission_client = self.create_client(SetCanState, '/ros_can/set_mission')
            self.create_timer(2.0, self._ensure_manual_mission)
        self.get_logger().info(f'Keyboard teleop, cmd_topic={cmd_topic}, mode {self.mode}')

    # ---------------------------------------------------------------- inputs
    def _img_cb(self, msg, topic=None):
        topic = topic or self.get_parameter('image_topic').value
        # Stick to the first topic that delivers, so two cameras do not flicker.
        if self._img_topic is None:
            self._img_topic = topic
            self.get_logger().info(f'Showing {topic}')
        if topic == self._img_topic and self.window:
            self.window.image_callback(msg)

    def _source_cb(self, msg):
        if not self._mux_seen:
            self._mux_seen = True
            if self._detect_mux:
                self.get_logger().info('cmd_mux found: publishing /falcon/cmd_teleop')
            if msg.data == 'auto' and not self._first_key and self.mode == 'MANUAL':
                self.mode = 'AUTO'
                self.get_logger().info('Planner is driving: starting in AUTO (M or W/A/S/D to take over)')
        self.control_source = msg.data

    def _can_state_cb(self, msg):
        self._ami_manual = msg.ami_state == CanState.AMI_MANUAL
        self.mission_text = 'manual drive' if self._ami_manual else f'ami {msg.ami_state}'

    def _ensure_manual_mission(self):
        """EUFS ignores /cmd until a mission is set; set AMI_MANUAL once if none is."""
        if self._ami_manual or not self._mission_client.service_is_ready():
            return
        req = SetCanState.Request()
        req.ami_state = CanState.AMI_MANUAL
        req.as_state = CanState.AS_OFF
        self._mission_client.call_async(req)

    def set_key(self, name, on):
        self.keys[name] = on
        self._first_key = True
        if on and self.mode == 'AUTO':
            self.mode = 'MANUAL'
            self.speed = 0.0
            self.get_logger().info('Manual takeover')
        if on and name == 'brake':
            self.cruise = False

    def toggle_cruise(self):
        self.cruise = not self.cruise
        if self.mode == 'AUTO' and self.cruise:
            self.mode = 'MANUAL'

    def change_cruise(self, sign):
        self.cruise_speed = min(self.max_speed, max(0.5, self.cruise_speed + sign * self.speed_step))

    def toggle_mode(self):
        self.mode = 'AUTO' if self.mode == 'MANUAL' else 'MANUAL'
        self.speed = 0.0
        self.cruise = False
        self.get_logger().info(f'Mode: {self.mode}')

    def stop(self):
        self.mode = 'MANUAL'
        self.cruise = False
        self.speed = 0.0
        self.steering = 0.0

    # ---------------------------------------------------------------- output
    def publish_cmd(self):
        now = time.monotonic()
        dt = min(0.1, now - self._last_t)
        self._last_t = now
        if self.mode != 'MANUAL':
            return   # AUTO: let cmd_mux pass the planner's commands
        if self._detect_mux and not self._mux_seen and now - self._t_start < 1.0:
            return   # give cmd_mux a moment to show up before writing /cmd directly

        k = self.keys
        if k['brake']:
            self.speed = max(0.0, self.speed - self.brake_decel * dt)
        elif k['forward'] or self.cruise:
            if self.speed < self.cruise_speed:
                self.speed = min(self.cruise_speed, self.speed + self.accel * dt)
            else:
                self.speed = max(self.cruise_speed, self.speed - self.coast * dt)
        else:
            self.speed = max(0.0, self.speed - self.coast * dt)

        if k['left'] and not k['right']:
            self.steering = min(self.max_steering, self.steering + self.steer_rate * dt)
        elif k['right'] and not k['left']:
            self.steering = max(-self.max_steering, self.steering - self.steer_rate * dt)
        elif self.steering > 0:
            self.steering = max(0.0, self.steering - self.steer_return * dt)
        else:
            self.steering = min(0.0, self.steering + self.steer_return * dt)

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steering)
        (self.pub_mux if self._detect_mux and self._mux_seen else self.pub).publish(msg)


def main(args=None):
    rclpy.init(args=args)
    if not HAS_TK:
        print('tkinter required (usually bundled with Python)')
        return 1
    if not HAS_IMAGE:
        print('Camera display requires: cv_bridge, opencv-python, Pillow. '
              'Install: sudo apt install ros-humble-cv-bridge; pip install opencv-python Pillow')
        return 1

    node = KeyboardTeleopNode()
    window = TeleopWindow(node)
    node.window = window

    # ROS executor in a background thread so the tk mainloop stays responsive
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    def on_closing():
        executor.shutdown()
        rclpy.try_shutdown()
        window.root.quit()

    window.root.protocol('WM_DELETE_WINDOW', on_closing)
    window.run()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
