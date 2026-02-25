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
    # Ultralytics supports YOLO26 weights like "yolo26n.pt" and auto-downloads them on first use.
    from ultralytics import YOLO
except Exception as exc:
    YOLO = None
    _ULTRALYTICS_IMPORT_ERROR = exc
else:
    _ULTRALYTICS_IMPORT_ERROR = None

try:
    from cv_bridge import CvBridge
except Exception as exc:
    CvBridge = None
    _CV_BRIDGE_IMPORT_ERROR = exc
else:
    _CV_BRIDGE_IMPORT_ERROR = None


class ObjectDetector:
    def __init__(self, node: Node, tf_buffer):
        self.minconf = 0.2
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

        # Topic resolution order:
        # 1) explicit rgb_topic/depth_topic (if provided)
        # 2) d435_color_topic/d435_depth_topic contract parameters
        # 3) D435 defaults used by our MuJoCo bridge
        d435_color_default = str(
            self.node.declare_parameter('d435_color_topic', '/intel/D435i/color').value
        )
        d435_depth_default = str(
            self.node.declare_parameter('d435_depth_topic', '/intel/D435i/depth').value
        )

        self.rgb_topic = str(
            self.node.declare_parameter('rgb_topic', d435_color_default).value
        )
        self.depth_topic = str(
            self.node.declare_parameter('depth_topic', d435_depth_default).value
        )

        # Keep backward compatibility with older configs while preferring D435 topics.
        if self.rgb_topic == '/rgbd_camera/image':
            self.rgb_topic = d435_color_default
        if self.depth_topic == '/rgbd_camera/depth_image':
            self.depth_topic = d435_depth_default

        self.yolo_output_topic = self.node.declare_parameter('yolo_output_topic', '/yolo_output/image').value
        self.detector_min_period_sec = float(
            self.node.declare_parameter('detector_min_period_sec', 0.25).value
        )

        self.node.get_logger().info(
            f"ObjectDetector topics: rgb={self.rgb_topic}, depth={self.depth_topic}, out={self.yolo_output_topic}"
        )

        self.last_detection_wall_time = 0.0
        self.last_rgb_rx_wall_time = None
        self.last_depth_rx_wall_time = None
        self.last_health_warn = 0.0

        self.model = None
        self.classes = []
        self.detector_ready = False
        self.bridge = None

        if CvBridge is None:
            self.node.get_logger().warn(
                f"cv_bridge unavailable ({_CV_BRIDGE_IMPORT_ERROR}); object detection disabled."
            )
            return

        self.bridge = CvBridge()

        # --- Detector model selection (YOLO26 via Ultralytics) ---
        # Default is the smallest YOLO26 Detect model for edge/CPU use.
        self.yolo_model_name = str(self.node.declare_parameter('yolo_model', 'yolo26n.pt').value)
        self.yolo_device = str(self.node.declare_parameter('yolo_device', 'cpu').value)
        self.yolo_imgsz = int(self.node.declare_parameter('yolo_imgsz', 640).value)

        if YOLO is None:
            self.node.get_logger().warn(
                f"ultralytics not available ({_ULTRALYTICS_IMPORT_ERROR}); object detection disabled."
            )
        else:
            try:
                self.model = YOLO(self.yolo_model_name)
                # names is a dict: {class_id: class_name}
                self.classes = [self.model.names[i] for i in sorted(self.model.names.keys())]
                self.detector_ready = True
                self.node.get_logger().info(
                    f"Loaded YOLO model: {self.yolo_model_name} on device={self.yolo_device}, imgsz={self.yolo_imgsz}"
                )
            except Exception as exc:
                self.node.get_logger().warn(
                    f"Failed to load YOLO model '{self.yolo_model_name}': {exc}; object detection disabled."
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

            # --- YOLO26 inference (Ultralytics) ---
            # Downstream semanticslam.py expects the *legacy* OpenCV-DNN YOLO output format:
            #   self.detections = [out]
            # where out is an Nx(5+num_classes) array and each row is:
            #   [cx, cy, w, h, obj, score_0, score_1, ...]
            # with cx/cy/w/h normalized to the source image size.
            results = self.model.predict(
                source=cv_image,
                imgsz=self.yolo_imgsz,
                conf=self.minconf,
                device=self.yolo_device,
                verbose=False,
            )
            self.last_detection_wall_time = time.time()

            num_classes = len(self.classes) if self.classes else (len(getattr(self.model, 'names', {})) or 0)
            det_rows = []

            if results:
                r0 = results[0]
                boxes = getattr(r0, 'boxes', None)
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()  # float
                    confs = boxes.conf.cpu().numpy()  # float
                    clss = boxes.cls.cpu().numpy().astype(int)

                    for (x1, y1, x2, y2), conf, cls_id in zip(xyxy, confs, clss):
                        conf = float(conf)
                        if conf <= self.minconf:
                            continue

                        # Normalize to legacy format expected by SemanticSLAM.
                        cx = float((x1 + x2) * 0.5 / max(1, self.width))
                        cy = float((y1 + y2) * 0.5 / max(1, self.height))
                        ww = float((x2 - x1) / max(1, self.width))
                        hh = float((y2 - y1) / max(1, self.height))
                        cls_id = int(cls_id)

                        row = np.zeros((5 + max(num_classes, cls_id + 1),), dtype=np.float32)
                        row[0] = cx
                        row[1] = cy
                        row[2] = ww
                        row[3] = hh
                        row[4] = conf  # objectness-ish
                        row[5 + cls_id] = conf  # per-class score used by SemanticSLAM
                        det_rows.append(row)

                        # Draw detections
                        ix1, iy1, ix2, iy2 = int(x1), int(y1), int(x2), int(y2)
                        cv2.rectangle(cv_image, (ix1, iy1), (ix2, iy2), (0, 255, 0), 2)
                        name = self.model.names.get(cls_id, f"id{cls_id}")
                        label = f"{name}: {conf:.2f}"
                        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        y_text = max(0, iy1 + 4 + th)
                        cv2.rectangle(
                            cv_image,
                            (ix1, max(0, y_text - th - 2)),
                            (ix1 + tw + 2, y_text + 2),
                            (0, 255, 0),
                            -1,
                        )
                        cv2.putText(
                            cv_image,
                            label,
                            (ix1 + 1, y_text),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 0),
                            1,
                            lineType=cv2.LINE_AA,
                        )

            det_arr = (
                np.stack(det_rows, axis=0)
                if det_rows
                else np.zeros((0, 5 + max(num_classes, 1)), dtype=np.float32)
            )
            with self.lock:
                # Keep the same shape contract as OpenCV-DNN: list of output arrays.
                self.detections = [det_arr]
            output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            output_msg.header = msg.header
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
