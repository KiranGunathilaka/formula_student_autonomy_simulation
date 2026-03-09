#!/usr/bin/env python3
"""
Keyboard teleop for EUFS simulation.
Game-style controls: W/S forward/brake, A/D steer, R/F speed +/-, Space stop.
Embedded front-view from ZED camera when available (launch_group:=default).
Uses tkinter (no PyQt) to avoid Qt/cv2 plugin conflicts.
"""
import sys

import rclpy
from rclpy.node import Node
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
except ImportError:
    CvBridge = None

try:
    import PIL.Image
    import PIL.ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

HAS_IMAGE = CvBridge is not None and HAS_PIL


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
            text='W: Forward  S: Brake  A/D: Steer  R: Speed+  F: Speed-  Space: Stop',
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

        # Image / placeholder - camera is optional; driving works without it
        self.img_frame = tk.Frame(self.root, bg='#2a2a2a', width=640, height=360)
        self.img_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.img_label = tk.Label(
            self.img_frame,
            text='Camera view (optional)\nClick this window to enable keyboard controls',
            fg='#888',
            bg='#2a2a2a',
            font=('', 11),
        )
        self.img_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        # bind_all so keys work even when child widgets have focus; include uppercase for Caps Lock
        for k, handler in [
            ('w', self._key_w), ('W', self._key_w), ('s', self._key_s), ('S', self._key_s),
            ('a', self._key_a), ('A', self._key_a), ('d', self._key_d), ('D', self._key_d),
            ('r', self._key_r), ('R', self._key_r), ('f', self._key_f), ('F', self._key_f),
        ]:
            self.root.bind_all(f'<KeyPress-{k}>', handler)
        for k, handler in [
            ('w', self._key_w_up), ('W', self._key_w_up), ('s', self._key_s_up), ('S', self._key_s_up),
            ('a', self._key_a_up), ('A', self._key_a_up), ('d', self._key_d_up), ('D', self._key_d_up),
        ]:
            self.root.bind_all(f'<KeyRelease-{k}>', handler)
        self.root.bind_all('<KeyPress-space>', self._key_space)

        # Ensure window can receive keys: focus on show and on any click
        def _take_focus(e=None):
            self.root.focus_force()
        self.root.after(100, _take_focus)
        self.root.bind('<FocusIn>', _take_focus)
        self.img_frame.bind('<Button-1>', _take_focus)
        self.img_label.bind('<Button-1>', _take_focus)

    def _key_w(self, e):
        self.node.set_forward(True)
    def _key_s(self, e):
        self.node.set_brake(True)
    def _key_a(self, e):
        self.node.set_steer_left(True)
    def _key_d(self, e):
        self.node.set_steer_right(True)
    def _key_r(self, e):
        self.node.speed_up()
    def _key_f(self, e):
        self.node.speed_down()
    def _key_space(self, e):
        self.node.stop()
    def _key_w_up(self, e):
        self.node.set_forward(False)
    def _key_s_up(self, e):
        self.node.set_brake(False)
    def _key_a_up(self, e):
        self.node.set_steer_left(False)
    def _key_d_up(self, e):
        self.node.set_steer_right(False)

    def image_callback(self, msg):
        if not HAS_IMAGE or self.bridge is None:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
            self.last_image = cv_img
            self._update_image()
        except Exception as e:
            self.node.get_logger().warn(f'Image conversion failed: {e}')

    def _update_image(self):
        if self.last_image is None:
            return
        try:
            pil_img = PIL.Image.fromarray(self.last_image)
            w, h = self.img_frame.winfo_width(), self.img_frame.winfo_height()
            if w < 10:
                w, h = 640, 360
            pil_img.thumbnail((w, h), PIL.Image.Resampling.LANCZOS)
            self.photo = PIL.ImageTk.PhotoImage(pil_img)
            self.img_label.configure(image=self.photo, text='')
            self.img_label.image = self.photo
        except Exception:
            pass

    def update_status(self):
        self.status_var.set(
            f'Speed target: {self.node.speed:.1f} m/s  |  Steer: {self.node.steering:.2f} rad'
        )

    def run(self):
        self.root.mainloop()


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('cmd_topic', '/cmd')
        self.declare_parameter('image_topic', '/zed/left/image_rect_color')
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
        self.timer = self.create_timer(1.0 / rate, self.publish_cmd)

        self._request_manual_drive()

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
            self.window.image_callback(msg)

    def _request_manual_drive(self):
        """Call /ros_can/set_mission to enable Manual Drive so /cmd is accepted."""
        client = self.create_client(SetCanState, '/ros_can/set_mission')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '/ros_can/set_mission not available; click Manual Drive in Mission Control'
            )
            return
        req = SetCanState.Request()
        req.ami_state = CanState.AMI_MANUAL
        req.as_state = CanState.AS_OFF
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info('Manual drive enabled')
            if f.result().success
            else self.get_logger().warn(f'Set mission failed: {f.result().message}')
        )

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
            target_speed = self.speed
        elif self.brake:
            target_speed = 0.0
        else:
            target_speed = 0.0

        if self.steer_left:
            self.steering = max(-self.max_steering, self.steering - self.steering_increment)
        elif self.steer_right:
            self.steering = min(self.max_steering, self.steering + self.steering_increment)
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
            self.window.update_status()


def main(args=None):
    rclpy.init(args=args)

    if not HAS_TK:
        print('tkinter required (usually bundled with Python)')
        return 1

    node = KeyboardTeleopNode()
    window = TeleopWindow(node)
    node.window = window

    # Integrate rclpy with tkinter: poll ROS in tk main loop
    def spin():
        if rclpy.ok() and window.root.winfo_exists():
            rclpy.spin_once(node, timeout_sec=0)
            window.root.after(10, spin)
        else:
            window.root.quit()

    window.root.protocol('WM_DELETE_WINDOW', lambda: (rclpy.shutdown(), window.root.quit()))
    window.root.after(10, spin)
    window.run()

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
