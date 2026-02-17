# yolo.py
import time
import os
import numpy as np
import cv2
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.time import Time

class ObjectDetector:
    def __init__(self, node: Node, tf_buffer):
        self.minconf = 0.1
        self.node = node
        self.tf_buffer = tf_buffer  # (not used directly here but available for later extensions)
        self.bridge = CvBridge()
        self.processing = False
        self.detections = None
        self.depth_image = None
        self.last_image_stamp = None
        self.lock = threading.Lock()
        # Parameters for image dimensions and focal lengths
        self.image_width = 320
        self.image_height = 240
        self.horizontal_fov = 1.25  # in radians
        self.fx = self.image_width / (2 * np.tan(self.horizontal_fov / 2))
        self.fy = self.image_height / (2 * np.tan(self.horizontal_fov / 2))
        # Topic and frame parameters are configurable for different robot stacks.
        self.rgb_topic = self.node.declare_parameter('rgb_topic', '/rgbd_camera/image').value
        self.depth_topic = self.node.declare_parameter('depth_topic', '/rgbd_camera/depth_image').value
        self.yolo_output_topic = self.node.declare_parameter('yolo_output_topic', '/yolo_output/image').value
        # Load YOLO model and classes from this module directory.
        module_dir = os.path.dirname(os.path.abspath(__file__))
        weights_path = os.path.join(module_dir, 'yolov4-tiny.weights')
        cfg_path = os.path.join(module_dir, 'yolov4-tiny.cfg')
        classes_path = os.path.join(module_dir, 'coco.names')
        self.net = None
        self.classes = []
        self.detector_ready = False
        if os.path.exists(weights_path) and os.path.exists(cfg_path) and os.path.exists(classes_path):
            self.net = cv2.dnn.readNet(weights_path, cfg_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            with open(classes_path, 'r') as f:
                self.classes = [line.strip() for line in f.readlines()]
            self.detector_ready = True
        else:
            self.node.get_logger().warn(
                "YOLO model files not found; object detection is disabled."
            )
        # Create subscribers and publisher using a QoS profile that keeps only the latest message.
        qos_profile_yolo = rclpy.qos.QoSProfile(depth=1)
        qos_profile_yolo.history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST
        qos_profile_yolo.durability = rclpy.qos.QoSDurabilityPolicy.VOLATILE
        self.image_sub = self.node.create_subscription(
            Image, self.rgb_topic, self.image_callback, qos_profile_yolo)
        self.depth_sub = self.node.create_subscription(
            Image, self.depth_topic, self.depth_callback, qos_profile_yolo)
        self.image_pub = self.node.create_publisher(Image, self.yolo_output_topic, qos_profile_yolo)

    def image_callback(self, msg):
        with self.lock:
            if self.depth_image is None:
                return  # Wait until a depth image is available
            if self.processing:
                return
            self.processing = True
        if not self.detector_ready:
            with self.lock:
                self.processing = False
            return
        self.last_image_stamp = Time.from_msg(msg.header.stamp)
        start_time = time.time()
        self.node.get_logger().info("Processing image for object detection")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.height, self.width, _ = cv_image.shape
        blob = cv2.dnn.blobFromImage(cv_image, 0.00392, (608, 608), (0, 0, 0), True, crop=False)
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers()]
        self.detections = self.net.forward(output_layers)
        # Process detections: draw bounding boxes and labels on the image.
        for out in self.detections:
            for detection in out:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = scores[class_id]
                if confidence > self.minconf:  # Adjust threshold as needed
                    center_x = int(detection[0] * self.width)
                    center_y = int(detection[1] * self.height)
                    w = int(detection[2] * self.width)
                    h = int(detection[3] * self.height)
                    cv2.rectangle(cv_image,
                                  (center_x - w // 2, center_y - h // 2),
                                  (center_x + w // 2, center_y + h // 2),
                                  (0, 255, 0), 2)
                    label = f"{self.classes[class_id]}: {confidence:.2f}"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.putText(cv_image, label,
                                (center_x - tw // 2, center_y - h // 2 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        self.image_pub.publish(output_msg)
        elapsed_time = time.time() - start_time
        self.node.get_logger().info(f"DONE detect: {elapsed_time:.2f} seconds")
        with self.lock:
            self.processing = False

    def depth_callback(self, msg):
        self.node.get_logger().info("Depth image received")
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
