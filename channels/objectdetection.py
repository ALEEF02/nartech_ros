#!/usr/bin/env python3
# yolo.py
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except Exception as exc:
    CvBridge = None
    _CV_BRIDGE_IMPORT_ERROR = exc
else:
    _CV_BRIDGE_IMPORT_ERROR = None


class ObjectDetector:
    def __init__(self, node: Node, tf_buffer):
        self.minconf = 0.1
        self.node = node
        self.tf_buffer = tf_buffer  # Reserved for future extensions.
        self.processing = False
        self.detections = None
        self.depth_image = None
        self.last_image_stamp = None
        self.lock = threading.Lock()

        self.image_width = 320
        self.image_height = 240
        self.horizontal_fov = 1.25  # radians
        self.fx = self.image_width / (2 * np.tan(self.horizontal_fov / 2))
        self.fy = self.image_height / (2 * np.tan(self.horizontal_fov / 2))

        self.rgb_topic = self.node.declare_parameter('rgb_topic', '/rgbd_camera/image').value
        self.depth_topic = self.node.declare_parameter('depth_topic', '/rgbd_camera/depth_image').value
        self.yolo_output_topic = self.node.declare_parameter('yolo_output_topic', '/yolo_output/image').value
        self.detector_min_period_sec = float(
            self.node.declare_parameter('detector_min_period_sec', 0.25).value
        )

        self.last_detection_wall_time = 0.0
        self.last_rgb_rx_wall_time = None
        self.last_depth_rx_wall_time = None
        self.last_health_warn = 0.0

        self.net = None
        self.classes = []
        self.detector_ready = False
        self.bridge = None

        if CvBridge is None:
            self.node.get_logger().warn(
                f"cv_bridge unavailable ({_CV_BRIDGE_IMPORT_ERROR}); object detection disabled."
            )
            return

        self.bridge = CvBridge()

        module_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(module_dir, 'yolov4-tiny.weights')
        cfg_path = os.path.join(module_dir, 'yolov4-tiny.cfg')
        classes_path = os.path.join(module_dir, 'coco.names')
        if os.path.exists(weights_path) and os.path.exists(cfg_path) and os.path.exists(classes_path):
            self.net = cv2.dnn.readNet(weights_path, cfg_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            with open(classes_path, 'r', encoding='utf-8') as f:
                self.classes = [line.strip() for line in f.readlines()]
            self.detector_ready = True
        else:
            self.node.get_logger().warn(
                "YOLO model files not found; object detection is disabled."
            )

        # Use sensor-data QoS so D435i best-effort publishers are compatible.
        self.image_sub = self.node.create_subscription(
            Image, self.rgb_topic, self.image_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.node.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_sensor_data
        )

        qos_profile_pub = QoSProfile(depth=1)
        qos_profile_pub.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_pub.durability = QoSDurabilityPolicy.VOLATILE
        qos_profile_pub.reliability = QoSReliabilityPolicy.RELIABLE
        self.image_pub = self.node.create_publisher(Image, self.yolo_output_topic, qos_profile_pub)
        self.health_timer = self.node.create_timer(5.0, self._health_check)

    def _health_check(self):
        now = time.time()
        if now - self.last_health_warn < 10.0:
            return
        if self.last_depth_rx_wall_time is None:
            self.node.get_logger().warn(
                f"No depth messages received yet on {self.depth_topic}. Check topic/QoS."
            )
            self.last_health_warn = now
        elif self.last_rgb_rx_wall_time is None:
            self.node.get_logger().warn(
                f"No RGB messages received yet on {self.rgb_topic}. Check topic/QoS."
            )
            self.last_health_warn = now

    def image_callback(self, msg):
        self.last_rgb_rx_wall_time = time.time()
        with self.lock:
            if self.depth_image is None or self.processing:
                return
            if (time.time() - self.last_detection_wall_time) < self.detector_min_period_sec:
                return
            self.processing = True
        try:
            if not self.detector_ready or self.bridge is None:
                return
            self.last_image_stamp = Time.from_msg(msg.header.stamp)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.height, self.width, _ = cv_image.shape
            if self.width > 0 and self.height > 0:
                self.fx = self.width / (2 * np.tan(self.horizontal_fov / 2))
                self.fy = self.height / (2 * np.tan(self.horizontal_fov / 2))
            blob = cv2.dnn.blobFromImage(
                cv_image, 0.00392, (608, 608), (0, 0, 0), swapRB=True, crop=False
            )
            self.net.setInput(blob)
            layer_names = self.net.getLayerNames()
            output_layers = [layer_names[i - 1] for i in np.array(self.net.getUnconnectedOutLayers()).flatten()]
            self.detections = self.net.forward(output_layers)
            self.last_detection_wall_time = time.time()
            for out in self.detections:
                for detection in out:
                    scores = detection[5:]
                    class_id = int(np.argmax(scores))
                    confidence = float(scores[class_id])
                    if confidence <= self.minconf:
                        continue
                    center_x = int(detection[0] * self.width)
                    center_y = int(detection[1] * self.height)
                    w = int(detection[2] * self.width)
                    h = int(detection[3] * self.height)
                    cv2.rectangle(
                        cv_image,
                        (center_x - w // 2, center_y - h // 2),
                        (center_x + w // 2, center_y + h // 2),
                        (0, 255, 0),
                        2,
                    )
                    label = f"{self.classes[class_id]}: {confidence:.2f}" if class_id < len(self.classes) else f"id{class_id}: {confidence:.2f}"
                    (tw, _), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.putText(
                        cv_image,
                        label,
                        (center_x - tw // 2, center_y - h // 2 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                    )
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.image_pub.publish(output_msg)
        except Exception as exc:
            self.node.get_logger().warn(f"Object detection callback error: {exc}")
        finally:
            with self.lock:
                self.processing = False

    def depth_callback(self, msg):
        self.last_depth_rx_wall_time = time.time()
        if self.bridge is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            # Unitree D435i sim uses 16UC1 (mm). Normalize to meters for semantic math.
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001
            elif depth.dtype != np.float32:
                depth = depth.astype(np.float32)
            with self.lock:
                self.depth_image = depth
        except Exception as exc:
            self.node.get_logger().warn(f"Depth callback conversion error: {exc}")
