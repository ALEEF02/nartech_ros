#!/usr/bin/env python3
"""Primary/secondary LaserScan selector with stale-source failover."""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class ScanMuxSelector(Node):
    def __init__(self):
        super().__init__('scan_mux_selector')

        self.primary_topic = self.declare_parameter(
            'scan_primary_topic', '/scan/livox'
        ).value
        self.secondary_topic = self.declare_parameter(
            'scan_secondary_topic', '/scan/depth'
        ).value
        self.output_topic = self.declare_parameter(
            'scan_output_topic', '/scan'
        ).value
        self.primary_stale_timeout = float(self.declare_parameter(
            'primary_stale_timeout_sec', 0.6
        ).value)
        self.secondary_stale_timeout = float(self.declare_parameter(
            'secondary_stale_timeout_sec', 1.0
        ).value)
        self.primary_reacquire_delay = float(self.declare_parameter(
            'primary_reacquire_delay_sec', 1.0
        ).value)
        self.publish_rate_hz = float(self.declare_parameter(
            'scan_publish_rate_hz', 20.0
        ).value)
        # Kept for backward-compatible config files; not used for gating.
        self.max_input_msg_age_sec = float(self.declare_parameter(
            'max_input_msg_age_sec', 1.0
        ).value)
        self.max_future_offset_sec = float(self.declare_parameter(
            'max_future_offset_sec', 0.25
        ).value)
        self.restamp_scan = _as_bool(self.declare_parameter(
            'restamp_scan', True
        ).value)
        self.scan_output_frame = self.declare_parameter(
            'scan_output_frame', ''
        ).value

        self.primary_msg = None
        self.secondary_msg = None
        self.primary_seen_time = None
        self.secondary_seen_time = None
        self.primary_seq = 0
        self.secondary_seq = 0
        self.primary_fresh_since = None
        self.active_source = None
        self.last_published_source = None
        self.last_published_seq = -1

        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.primary_sub = self.create_subscription(
            LaserScan, self.primary_topic, self._primary_cb, qos_profile_sensor_data
        )
        self.secondary_sub = self.create_subscription(
            LaserScan, self.secondary_topic, self._secondary_cb, qos_profile_sensor_data
        )

        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self._timer_cb)
        self.get_logger().info(
            "Scan selector active: "
            f"primary={self.primary_topic}, secondary={self.secondary_topic}, out={self.output_topic}, "
            f"restamp={self.restamp_scan}, frame_override='{self.scan_output_frame}'"
        )

    def _primary_cb(self, msg: LaserScan):
        self.primary_msg = msg
        self.primary_seen_time = self.get_clock().now()
        self.primary_seq += 1

    def _secondary_cb(self, msg: LaserScan):
        self.secondary_msg = msg
        self.secondary_seen_time = self.get_clock().now()
        self.secondary_seq += 1

    def _is_fresh(self, seen_time, timeout_sec: float) -> bool:
        if seen_time is None:
            return False
        age = self.get_clock().now() - seen_time
        return age <= Duration(seconds=timeout_sec)

    def _timer_cb(self):
        primary_fresh = self._is_fresh(self.primary_seen_time, self.primary_stale_timeout)
        secondary_fresh = self._is_fresh(self.secondary_seen_time, self.secondary_stale_timeout)

        if self.active_source in (None, 'primary'):
            if primary_fresh:
                self.active_source = 'primary'
            elif secondary_fresh:
                self.active_source = 'secondary'
                self.get_logger().warn("Primary scan stale, switching to secondary scan.")
        elif self.active_source == 'secondary':
            if primary_fresh:
                if not secondary_fresh:
                    self.active_source = 'primary'
                    self.primary_fresh_since = None
                    self.get_logger().info("Secondary scan stale, switching immediately to primary.")
                elif self.primary_fresh_since is None:
                    self.primary_fresh_since = self.get_clock().now()
                elif (self.get_clock().now() - self.primary_fresh_since) >= Duration(
                    seconds=self.primary_reacquire_delay
                ):
                    self.active_source = 'primary'
                    self.primary_fresh_since = None
                    self.get_logger().info("Primary scan recovered, switching back to primary.")
            else:
                self.primary_fresh_since = None
                if not secondary_fresh:
                    self.active_source = None
                    self.get_logger().warn("Both scan sources stale; publishing paused.")

        if self.active_source == 'primary':
            self._publish_if_new('primary', self.primary_msg, self.primary_seq, primary_fresh)
        elif self.active_source == 'secondary':
            self._publish_if_new('secondary', self.secondary_msg, self.secondary_seq, secondary_fresh)

    def _publish_if_new(self, source: str, msg: LaserScan, seq: int, source_fresh: bool):
        if not source_fresh or msg is None:
            return
        # Avoid flooding Nav2/SLAM with duplicate scans when upstream is unchanged.
        if self.last_published_source == source and self.last_published_seq == seq:
            return
        self.scan_pub.publish(self._prepared_scan(msg))
        self.last_published_source = source
        self.last_published_seq = seq

    def _prepared_scan(self, msg: LaserScan) -> LaserScan:
        # Rebuild with fresh header timing/frame to keep downstream TF filters stable.
        out = LaserScan()
        out.header = msg.header
        if self.restamp_scan:
            out.header.stamp = self.get_clock().now().to_msg()
        if self.scan_output_frame:
            out.header.frame_id = self.scan_output_frame
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = msg.ranges
        out.intensities = msg.intensities
        return out


def main(args=None):
    rclpy.init(args=args)
    node = ScanMuxSelector()
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
