#!/usr/bin/env python3
"""
Command mux: one path to /cmd for both the autonomy stack and the keyboard.

  /falcon/cmd_teleop  (keyboard_teleop, only while it is in MANUAL mode)  --+
                                                                             +--> /cmd
  /falcon/cmd_auto    (pure_pursuit)  ---------------------------------------+

Teleop wins whenever it has published within `teleop_timeout_s`; otherwise the
autonomy command passes through.  keyboard_teleop stops publishing when you switch
it back to AUTO (key M), so control returns to the planner on its own.

Publishes the active source on /falcon/control_source (std_msgs/String:
teleop | auto | idle) for the teleop window and RViz.
"""

import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from std_msgs.msg import String


class CmdMux(Node):
    def __init__(self):
        super().__init__('cmd_mux')
        self.declare_parameter('auto_topic', '/falcon/cmd_auto')
        self.declare_parameter('teleop_topic', '/falcon/cmd_teleop')
        self.declare_parameter('output_topic', '/cmd')
        self.declare_parameter('teleop_timeout_s', 0.3)
        self.declare_parameter('auto_timeout_s', 0.5)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._teleop_timeout = float(g('teleop_timeout_s'))
        self._auto_timeout = float(g('auto_timeout_s'))
        self._t_teleop = -1e9
        self._t_auto = -1e9
        self._pub = self.create_publisher(AckermannDriveStamped, g('output_topic'), 10)
        self._src_pub = self.create_publisher(String, '/falcon/control_source', 10)
        self.create_subscription(AckermannDriveStamped, g('auto_topic'), self._auto_cb, 10)
        self.create_subscription(AckermannDriveStamped, g('teleop_topic'), self._teleop_cb, 10)
        self.create_timer(0.2, self._publish_source)
        self._source = 'idle'
        self.get_logger().info(
            f"cmd_mux: {g('teleop_topic')} (priority) / {g('auto_topic')} -> {g('output_topic')}")

    # Wall time on purpose: teleop is a person at a keyboard, not a sim-time process.
    def _teleop_cb(self, msg):
        self._t_teleop = time.monotonic()
        self._pub.publish(msg)

    def _auto_cb(self, msg):
        self._t_auto = time.monotonic()
        if time.monotonic() - self._t_teleop > self._teleop_timeout:
            self._pub.publish(msg)

    def _publish_source(self):
        now = time.monotonic()
        if now - self._t_teleop <= self._teleop_timeout:
            src = 'teleop'
        elif now - self._t_auto <= self._auto_timeout:
            src = 'auto'
        else:
            src = 'idle'
        if src != self._source:
            self.get_logger().info(f'control source: {src}')
            self._source = src
        self._src_pub.publish(String(data=src))


def main(args=None):
    rclpy.init(args=args)
    node = CmdMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # A callback that runs while Ctrl-C shuts the context down raises here.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
