#!/usr/bin/env python3
# main.py
import rclpy
from rclpy.node import Node
import tf2_ros
from channels.objectdetection import ObjectDetector
from channels.localization import Localization
from channels.semanticslam import SemanticSLAM
from channels.navigation import Navigation
from rclpy.duration import Duration

class MainNode(Node):
    def __init__(self):
        super().__init__('NARTECH_node')
        # Create a single TF2 buffer and listener to be shared across modules.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        # Instantiate the helper modules.
        self.pick = lambda x: None #overridden
        self.drop = lambda x: None #overridden
        self.object_detector = ObjectDetector(self, self.tf_buffer)
        self.localization = Localization(self, self.tf_buffer)
        self.semantic_slam = SemanticSLAM(self, self.tf_buffer, self.localization, self.object_detector)
        self.navigation = Navigation(self, self.semantic_slam, self.localization)
        self.start_navigation_to_coordinate = self.navigation.start_navigation_to_coordinate
        self.enable_arm_controller = bool(self.declare_parameter('enable_arm_controller', True).value)
        if self.enable_arm_controller:
            from channels.armcontroller import ArmController
            self.arm_controller = ArmController(self, self.semantic_slam, self.navigation)
            if getattr(self.arm_controller, 'available', True):
                self.pick = self.arm_controller.pick
                self.drop = self.arm_controller.drop
            else:
                self.arm_controller = None
                self.get_logger().warn("ArmController unavailable; continuing without manipulation.")
        else:
            self.arm_controller = None
            self.get_logger().info("ArmController disabled by parameter: enable_arm_controller:=False")

def main(args=None):
    rclpy.init(args=args)
    node = MainNode()
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
