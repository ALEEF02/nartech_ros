#!/usr/bin/env python3
"""Republish canonical /cmd_vel into a Unitree-specific command topic."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelAdapter(Node):
    def __init__(self):
        super().__init__('cmd_vel_adapter')
        self.cmd_vel_in = self.declare_parameter('cmd_vel_in', '/cmd_vel').value
        self.cmd_vel_out = self.declare_parameter('cmd_vel_out', '/unitree/cmd_vel').value
        self._pub = self.create_publisher(Twist, self.cmd_vel_out, 10)
        self._sub = self.create_subscription(Twist, self.cmd_vel_in, self._cb, 10)
        self.get_logger().info(
            f"CmdVel adapter active: {self.cmd_vel_in} -> {self.cmd_vel_out}"
        )

    def _cb(self, msg: Twist):
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
