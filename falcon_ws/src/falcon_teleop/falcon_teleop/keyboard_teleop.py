#!/usr/bin/env python3
"""
Keyboard teleop for EUFS simulation.
Game-style controls: W/S forward/brake, A/D steer, R/F speed +/-, Space stop.
Embedded front-view from teleop camera when available.
Uses tkinter (no PyQt) to avoid Qt/cv2 plugin conflicts.
"""
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Image
from eufs_msgs.srv import SetCanState
from eufs_msgs.msg import CanState

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

# Image topics: teleop_camera is always-on (any launch_group); ZED only with launch_group:=default
IMAGE_TOPIC_FALLBACKS = [
    '/teleop_camera/image_raw',       # always-on teleop camera
    '/eufs/teleop_camera/image_raw',  # if spawned with namespace
    '/zed/left/image_rect_color',
    '/zed/zed_left/image_raw',
]


class TeleopWindow:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Falcon Teleop')
        self.root.geometry('800x550')
        self.root.configure(bg='#1e1e1e')

        self.bridge = CvBridge() if HAS_IMAGE else None
        self.last_image = None
        self.photo = None

        # Key bindings
        bfont = tkfont.Font(size=11, weight='bold')
        lbl = tk.Label(
            self.root,
            text='W: Fwd  S: Brake  A/D: Steer  R/F: Speed  Space: Stop',
            font=bfont,
            fg='#eee',
            bg='#1e1e1e',
        )
        lbl.pack(pady=5)

        self.status_var = tk.StringVar(value='Speed: 0  |  Steer: 0')
        st_lbl = tk.Label(
            self.root, textvariable=self.status_var, fg='#aaa', bg='#1e1e1e'
        )
        st_lbl.pack(pady=2)

        # Image area - camera feed is required for teleop
        self.img_frame = tk.Frame(self.root, bg='#2a2a2a', width=640, height=360)
        self.img_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.img_frame.pack_propagate(False)  # keep frame size for proper layout
        self.img_canvas = tk.Canvas(
            self.img_frame,
            bg='#2a2a2a',
            highlightthickness=0,
            width=640,
            height=360,
        )
        self.img_canvas.pack(fill=tk.BOTH, expand=True)
        self.img_canvas.create_text(
            320, 180,
            text='Waiting for camera...\nClick this window to enable keyboard controls',
            fill='#888',
            font=('', 11),
            tags='placeholder',
        )

        # Single <Key> handler - more reliable across focus/display managers
        self.root.bind_all('<KeyPress>', self._on_key_press)
        self.root.bind_all('<KeyRelease>', self._on_key_release)

        # Ensure window can receive keys: focus on show and on any click
        def _take_focus(e=None):
            self.root.focus_force()
        self.root.after(100, _take_focus)
        self.root.bind('<FocusIn>', _take_focus)
        self.img_frame.bind('<Button-1>', _take_focus)
        self.img_canvas.bind('<Button-1>', _take_focus)
        self._canvas_image_id = None

    def _on_key_press(self, e):
        k = (e.keysym or '').lower()
        if k == 'w':
            self.node.set_forward(True)
        elif k == 's':
            self.node.set_brake(True)
        elif k == 'a':
            self.node.set_steer_left(True)
        elif k == 'd':
            self.node.set_steer_right(True)
        elif k == 'r':
            self.node.speed_up()
        elif k == 'f':
            self.node.speed_down()
        elif e.keysym == 'space':
            self.node.stop()

    def _on_key_release(self, e):
        k = (e.keysym or '').lower()
        if k == 'w':
            self.node.set_forward(False)
        elif k == 's':
            self.node.set_brake(False)
        elif k == 'a':
            self.node.set_steer_left(False)
        elif k == 'd':
            self.node.set_steer_right(False)

    def image_callback(self, msg):
        if not HAS_IMAGE or self.bridge is None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if cv_img is None or cv_img.size == 0:
                return
            # Ensure uint8 RGB
            if len(cv_img.shape) == 2:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2RGB)
            elif cv_img.shape[2] == 3:
                enc = (msg.encoding or 'bgr8').strip().lower()
                if enc in ('bgr8', '8uc3', 'bgra8'):
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                elif enc != 'rgb8':
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            self.last_image = np.ascontiguousarray(cv_img)
            self.root.after(0, self._update_image)
        except Exception as e:
            self.node.get_logger().warn(f'Image conversion failed: {e}')

    def _update_image(self):
        if self.last_image is None or not self.root.winfo_exists():
            return
        try:
            self.img_canvas.delete('placeholder')
            pil_img = PIL.Image.fromarray(self.last_image)
            cw = self.img_canvas.winfo_width()
            ch = self.img_canvas.winfo_height()
            if cw < 10 or ch < 10:
                cw, ch = 640, 360
            # Resampling.LANCZOS is Pillow 9.1+; fallback to LANCZOS/ANTIALIAS on older
            try:
                resample = PIL.Image.Resampling.LANCZOS
            except AttributeError:
                resample = getattr(PIL.Image, 'LANCZOS', PIL.Image.ANTIALIAS)
            pil_img.thumbnail((cw, ch), resample)
            self.photo = PIL.ImageTk.PhotoImage(pil_img)
            if self._canvas_image_id is not None:
                self.img_canvas.delete(self._canvas_image_id)
            self._canvas_image_id = self.img_canvas.create_image(
                cw // 2, ch // 2, image=self.photo, anchor=tk.CENTER
            )
            self._current_photo = self.photo  # keep ref to prevent GC
        except Exception as e:
            self.node.get_logger().warn(f'Image display failed: {e}')

    def update_status(self):
        if not self.root.winfo_exists():
            return
        self.status_var.set(
            f'Speed target: {self.node.speed:.1f} m/s  |  Steer: {self.node.steering:.2f} rad'
        )

    def run(self):
        self.root.mainloop()


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_topic', '/cmd')
        self.declare_parameter('image_topic', '/teleop_camera/image_raw')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('speed_increment', 1.0)
        self.declare_parameter('steering_increment', 0.1)
        self.declare_parameter('max_speed', 10.0)
        self.declare_parameter('min_speed', 0.0)
        self.declare_parameter('max_steering', 0.42)

        cmd_topic = self.get_parameter('cmd_topic').value
        image_topic = self.get_parameter('image_topic').value
        rate = self.get_parameter('publish_rate').value

        self.pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self.img_sub = self.create_subscription(
            Image, image_topic, self._img_cb, 10
        )
        # Subscribe to fallback topics in case primary uses different name
        for fallback in IMAGE_TOPIC_FALLBACKS:
            if fallback != image_topic:
                self.create_subscription(Image, fallback, self._img_cb, 10)
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)
        self._manual_drive_enabled = False
        self._manual_drive_timer = self.create_timer(2.0, self._retry_manual_drive)

        self.speed_increment = self.get_parameter('speed_increment').value
        self.steering_increment = self.get_parameter('steering_increment').value
        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_steering = self.get_parameter('max_steering').value

        self.speed = 0.0
        self.steering = 0.0
        self.forward = False
        self.brake = False
        self.steer_left = False
        self.steer_right = False

        self.window = None

    def _img_cb(self, msg):
        if self.window:
            if not getattr(self, '_img_received', False):
                self._img_received = True
                self.get_logger().info('Receiving camera images')
            self.window.image_callback(msg)

    def _retry_manual_drive(self):
        """Retry /ros_can/set_mission until Manual Drive is enabled (so /cmd is accepted)."""
        if self._manual_drive_enabled:
            return
        client = self.create_client(SetCanState, '/ros_can/set_mission')
        if not client.wait_for_service(timeout_sec=0.5):
            return
        req = SetCanState.Request()
        req.ami_state = CanState.AMI_MANUAL
        req.as_state = CanState.AS_OFF
        future = client.call_async(req)
        future.add_done_callback(self._on_set_mission_done)

    def _on_set_mission_done(self, future):
        try:
            result = future.result()
            if result.success:
                self._manual_drive_enabled = True
                self.get_logger().info('Manual drive enabled')
            else:
                self.get_logger().warn(f'Set mission failed: {result.message}')
        except Exception as e:
            self.get_logger().warn(f'Set mission call failed: {e}')

    def set_forward(self, on):
        self.forward = on

    def set_brake(self, on):
        self.brake = on

    def set_steer_left(self, on):
        self.steer_left = on

    def set_steer_right(self, on):
        self.steer_right = on

    def speed_up(self):
        self.speed = min(self.max_speed, self.speed + self.speed_increment)
        self.get_logger().info(f'Speed: {self.speed:.1f}')

    def speed_down(self):
        self.speed = max(self.min_speed, self.speed - self.speed_increment)
        self.get_logger().info(f'Speed: {self.speed:.1f}')

    def stop(self):
        self.speed = 0.0
        self.steering = 0.0
        self.get_logger().info('Stop')

    def publish_cmd(self):
        if self.forward:
            target_speed = max(0.0, self.speed)
        elif self.brake:
            target_speed = 0.0
        else:
            target_speed = 0.0

        # A = steer left (positive), D = steer right (negative)
        if self.steer_left:
            self.steering = min(self.max_steering, self.steering + self.steering_increment)
        elif self.steer_right:
            self.steering = max(-self.max_steering, self.steering - self.steering_increment)
        else:
            self.steering *= 0.85
            if abs(self.steering) < 0.01:
                self.steering = 0.0

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.drive.speed = float(target_speed)
        msg.drive.acceleration = 0.0
        msg.drive.steering_angle = float(self.steering)
        msg.drive.steering_angle_velocity = 0.0
        self.pub.publish(msg)

        if self.window:
            self.window.root.after(0, self.window.update_status)


def main(args=None):
    rclpy.init(args=args)

    if not HAS_TK:
        print('tkinter required (usually bundled with Python)')
        return 1
    if not HAS_IMAGE:
        print('Camera display requires: cv_bridge, opencv-python, Pillow. Install: sudo apt install ros-humble-cv-bridge; pip install opencv-python Pillow')
        return 1

    node = KeyboardTeleopNode()
    window = TeleopWindow(node)
    node.window = window

    # Run ROS executor in background thread so tk mainloop stays responsive
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    def on_closing():
        rclpy.shutdown()
        executor.shutdown()
        window.root.quit()

    window.root.protocol('WM_DELETE_WINDOW', on_closing)
    window.run()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
