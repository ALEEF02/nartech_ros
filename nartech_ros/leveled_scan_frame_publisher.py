#!/usr/bin/env python3
"""Publish a leveled scan frame that tracks base yaw, not roll/pitch."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class LeveledScanFramePublisher(Node):
    def __init__(self):
        super().__init__('leveled_scan_frame_publisher')

        self.odom_frame = str(self.declare_parameter('odom_frame', 'odom').value)
        self.base_frame = str(self.declare_parameter('base_frame', 'base_link').value)
        self.scan_level_frame = str(
            self.declare_parameter('scan_level_frame', 'base_scan_level').value
        )
        self.publish_rate_hz = max(
            1.0, float(self.declare_parameter('publish_rate_hz', 50.0).value)
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._last_warn_time = self.get_clock().now()

        period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(period, self._timer_cb)

        self.get_logger().info(
            "Leveled scan TF publisher active: "
            f"{self.odom_frame} -> {self.scan_level_frame} from source {self.odom_frame} -> {self.base_frame}, "
            f"rate={self.publish_rate_hz:.1f}Hz"
        )

    def _timer_cb(self):
        try:
            source_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception as exc:  # noqa: BLE001
            self._warn_throttled(
                f"Waiting for transform {self.odom_frame} -> {self.base_frame}: {exc}"
            )
            return

        q = source_tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        half_yaw = 0.5 * yaw

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.scan_level_frame
        out.transform.translation = source_tf.transform.translation
        out.transform.rotation.x = 0.0
        out.transform.rotation.y = 0.0
        out.transform.rotation.z = math.sin(half_yaw)
        out.transform.rotation.w = math.cos(half_yaw)

        self.tf_broadcaster.sendTransform(out)

    def _warn_throttled(self, message: str):
        now = self.get_clock().now()
        if (now - self._last_warn_time) < Duration(seconds=2.0):
            return
        self._last_warn_time = now
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = LeveledScanFramePublisher()
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
