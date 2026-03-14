# semanticslam.py
import cv2
import tf2_geometry_msgs
import time
import os
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from rclpy.time import Time
import tf2_ros
import math

from channels.exploration import shift_seen_grid
from mettabridge import space_tick


def _get_or_declare_parameter(node, name, default):
    if node.has_parameter(name):
        return node.get_parameter(name).value
    return node.declare_parameter(name, default).value


class SemanticSLAM:
    def __init__(self, node, tf_buffer, localization, object_detector):
        self.node = node
        self.tf_buffer = tf_buffer
        self.localization = localization
        self.object_detector = object_detector
        self.map_frame = _get_or_declare_parameter(self.node, 'map_frame', 'map')
        self.base_frame = _get_or_declare_parameter(self.node, 'base_frame', 'base_link')
        self.camera_frame = _get_or_declare_parameter(
            self.node, 'camera_frame', 'd435i_depth_cam_optical'
        )
        if self.camera_frame == 'oakd_left_camera_frame':
            self.node.get_logger().warn(
                "camera_frame=oakd_left_camera_frame is deprecated; using d435i_depth_cam_optical."
            )
            self.camera_frame = 'd435i_depth_cam_optical'
        self.map_topic = _get_or_declare_parameter(self.node, 'map_topic', '/map')
        self.lowres_map_topic = _get_or_declare_parameter(self.node, 'lowres_map_topic', '/lowres_map')
        self.lowres_seen_map_topic = _get_or_declare_parameter(
            self.node, 'lowres_seen_map_topic', '/lowres_seen_map'
        )
        self.seen_depth_sample_stride = max(
            1, int(_get_or_declare_parameter(self.node, 'seen_depth_sample_stride', 8))
        )
        self.seen_depth_min_m = float(_get_or_declare_parameter(self.node, 'seen_depth_min_m', 0.2))
        self.seen_depth_max_m = float(_get_or_declare_parameter(self.node, 'seen_depth_max_m', 4.0))
        if self.seen_depth_min_m > self.seen_depth_max_m:
            self.node.get_logger().warn(
                "seen_depth_min_m > seen_depth_max_m; swapping values."
            )
            self.seen_depth_min_m, self.seen_depth_max_m = self.seen_depth_max_m, self.seen_depth_min_m
        self.grid_dump_path = _get_or_declare_parameter(
            self.node,
            'grid_dump_path',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'grid.txt')
        )
        self.M = {
            "wall": 100,
            "robot": 127,
            "chair": -120,
            "bench": -126,
            "dining_table": -126,
            "bottle": -125,
            "cup": -125,
            "can": -125,
            "person": -124,
            "fridge": -123,
            "sink": -122,
            "stove": -121,
            "frisbee": -123,
            "orangeball": -123,
            "unknown": -1,
        }
        self.previous_detections_persistence = 100000.0  # seconds
        self.previous_detections = {}
        self.previous_detections_truth = {}
        self.downsample_factor = int(_get_or_declare_parameter(self.node, 'downsample_factor', 28))
        qos_profile_map = QoSProfile(depth=1)
        qos_profile_map.history = QoSHistoryPolicy.KEEP_LAST
        qos_profile_map.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.map_sub = self.node.create_subscription(
            OccupancyGrid, self.map_topic, self.occ_grid_callback, qos_profile_map
        )
        self.lowres_grid_pub = self.node.create_publisher(
            OccupancyGrid, self.lowres_map_topic, qos_profile_map
        )
        self.lowres_seen_grid_pub = self.node.create_publisher(
            OccupancyGrid, self.lowres_seen_map_topic, qos_profile_map
        )
        self.timer = self.node.create_timer(2.0, self.build_grid_periodic)
        self.cached_msg = None
        self.low_res_grid = None
        self.camera_seen_grid = None
        self.robot_lowres_x = None
        self.robot_lowres_y = None
        self.new_width = 0
        self.new_height = 0
        self.new_resolution = None
        self.origin = None
        self.mapupdate = 0
        self.goalstart = 0
        self.inventory = []

    def _sanitize_truth_confidence(self, value, default=0.9):
        try:
            confidence = float(value)
        except (TypeError, ValueError):
            return default
        if not math.isfinite(confidence):
            return default
        return max(0.0, min(1.0, confidence))

    def _project_pixel_to_camera_point(
        self, center_x, center_y, depth_value, width=None, height=None, fx=None, fy=None
    ):
        depth_value = float(depth_value)
        width = float(width if width is not None else getattr(self.object_detector, "width", 1))
        height = float(height if height is not None else getattr(self.object_detector, "height", 1))
        fx = max(1e-6, float(fx if fx is not None else self.object_detector.fx))
        fy = max(1e-6, float(fy if fy is not None else self.object_detector.fy))
        u = float(center_x) - (width / 2.0)
        v = float(center_y) - (height / 2.0)
        x_from_center = (u * depth_value) / fx
        y_from_center = (v * depth_value) / fy
        if "optical" in str(self.camera_frame).lower():
            return Point(x=x_from_center, y=y_from_center, z=depth_value)
        return Point(x=depth_value, y=-x_from_center, z=-y_from_center)

    def _normalize_detection_category(self, category):
        if category is None:
            return "unknown"
        return "_".join(str(category).strip().split())

    def _is_orange_crop(self, bgr_image):
        if bgr_image is None or bgr_image.size == 0:
            return False
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask_a = cv2.inRange(hsv_image, (5, 80, 60), (20, 255, 255))
        mask_b = cv2.inRange(hsv_image, (0, 80, 60), (5, 255, 255))
        mask = cv2.bitwise_or(mask_a, mask_b)
        orange_ratio = float(np.count_nonzero(mask)) / float(mask.size)
        return orange_ratio >= 0.20

    def _canonicalize_detection_category(self, category_raw, detection, rgb_snapshot, detector_width, detector_height):
        category = self._normalize_detection_category(category_raw)
        if category not in ("sports_ball", "orange"):
            return category
        if rgb_snapshot is None:
            return category
        rgb_image = rgb_snapshot.get("rgb_image")
        if rgb_image is None or rgb_image.size == 0:
            return category
        x_center = int(detection[0] * detector_width)
        y_center = int(detection[1] * detector_height)
        box_width = max(1, int(detection[2] * detector_width))
        box_height = max(1, int(detection[3] * detector_height))
        x0 = max(0, x_center - box_width // 2)
        y0 = max(0, y_center - box_height // 2)
        x1 = min(rgb_image.shape[1], x_center + box_width // 2)
        y1 = min(rgb_image.shape[0], y_center + box_height // 2)
        if x1 <= x0 or y1 <= y0:
            return category
        crop = rgb_image[y0:y1, x0:x1]
        if self._is_orange_crop(crop):
            return "orangeball"
        return category

    def _is_cell_in_bounds(self, x, y):
        return 0 <= x < self.new_width and 0 <= y < self.new_height

    def _mark_seen_cell(self, x, y, mark_robot=False):
        if not self._is_cell_in_bounds(x, y) or self.camera_seen_grid is None:
            return
        idx = y * self.new_width + x
        if mark_robot:
            self.camera_seen_grid[idx] = 127
            return
        if self.camera_seen_grid[idx] == -1:
            self.camera_seen_grid[idx] = 0

    def _bresenham_cells(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x = x0
        y = y0
        while True:
            yield x, y
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy

    def _mark_seen_ray(self, start_x, start_y, end_x, end_y):
        if self.camera_seen_grid is None:
            return
        for x, y in self._bresenham_cells(start_x, start_y, end_x, end_y):
            if not self._is_cell_in_bounds(x, y):
                break
            self._mark_seen_cell(x, y)
            idx = y * self.new_width + x
            if self.low_res_grid is not None and self.low_res_grid[idx] == 100:
                break

    def _quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        xx = qx * qx
        yy = qy * qy
        zz = qz * qz
        xy = qx * qy
        xz = qx * qz
        yz = qy * qz
        wx = qw * qx
        wy = qw * qy
        wz = qw * qz
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float64,
        )

    def _merge_previous_seen_grid(
        self,
        previous_seen_grid,
        previous_width,
        previous_height,
        previous_origin,
        previous_resolution,
        current_origin,
    ):
        if (
            previous_seen_grid is None
            or previous_origin is None
            or previous_resolution is None
            or previous_width <= 0
            or previous_height <= 0
        ):
            return [-1] * (self.new_width * self.new_height)
        if not math.isclose(previous_resolution, self.new_resolution, rel_tol=1e-6, abs_tol=1e-8):
            return [-1] * (self.new_width * self.new_height)
        offset_x = int(
            round((previous_origin.position.x - current_origin.position.x) / self.new_resolution)
        )
        offset_y = int(
            round((previous_origin.position.y - current_origin.position.y) / self.new_resolution)
        )
        return shift_seen_grid(
            previous_seen_grid,
            previous_width,
            previous_height,
            self.new_width,
            self.new_height,
            offset_x,
            offset_y,
            unseen_value=-1,
        )

    def _update_camera_seen_from_depth(self, original_origin, depth_snapshot):
        if depth_snapshot is None or self.camera_seen_grid is None:
            return
        depth_image = depth_snapshot.get("depth_image")
        if depth_image is None or getattr(depth_image, "ndim", 0) < 2:
            return
        depth_height = int(depth_snapshot.get("height", depth_image.shape[0]))
        depth_width = int(depth_snapshot.get("width", depth_image.shape[1]))
        depth_height = max(0, min(depth_height, depth_image.shape[0]))
        depth_width = max(0, min(depth_width, depth_image.shape[1]))
        if depth_height <= 0 or depth_width <= 0:
            return
        fx = float(depth_snapshot.get("fx", self.object_detector.fx))
        fy = float(depth_snapshot.get("fy", self.object_detector.fy))
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.camera_frame, Time(), timeout=Duration(seconds=1.0)
            )
        except Exception as exc:
            self.node.get_logger().warn(f"Unable to update camera seen grid, TF error: {exc}")
            return
        camera_world_x = transform.transform.translation.x
        camera_world_y = transform.transform.translation.y
        camera_grid_x, camera_grid_y = self.get_lowres_position(
            camera_world_x, camera_world_y, original_origin, self.new_resolution
        )
        if not self._is_cell_in_bounds(camera_grid_x, camera_grid_y):
            return
        self._mark_seen_cell(camera_grid_x, camera_grid_y)
        q = transform.transform.rotation
        rotation_matrix = self._quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float64,
        )
        for center_y in range(0, depth_height, self.seen_depth_sample_stride):
            for center_x in range(0, depth_width, self.seen_depth_sample_stride):
                depth_value = float(depth_image[center_y, center_x])
                if (
                    not math.isfinite(depth_value)
                    or depth_value < self.seen_depth_min_m
                    or depth_value > self.seen_depth_max_m
                ):
                    continue
                camera_point = self._project_pixel_to_camera_point(
                    center_x,
                    center_y,
                    depth_value,
                    width=depth_width,
                    height=depth_height,
                    fx=fx,
                    fy=fy,
                )
                camera_vector = np.array(
                    [camera_point.x, camera_point.y, camera_point.z], dtype=np.float64
                )
                world_vector = rotation_matrix.dot(camera_vector) + translation
                object_grid_x, object_grid_y = self.get_lowres_position(
                    world_vector[0], world_vector[1], original_origin, self.new_resolution
                )
                if not self._is_cell_in_bounds(object_grid_x, object_grid_y):
                    continue
                self._mark_seen_ray(camera_grid_x, camera_grid_y, object_grid_x, object_grid_y)

    def occ_grid_callback(self, msg):
        self.node.get_logger().info("NEW OCC GRID")
        self.cached_msg = msg
        self.build_grid_periodic()

    def build_grid_periodic(self):
        if self.cached_msg is None:
            self.node.get_logger().info("WAITING FOR OCC GRID")
            return
        start_time = time.time()
        self.node.get_logger().info("NEW PROCESSING: OCC GRID")
        previous_seen_grid = self.camera_seen_grid
        previous_width = self.new_width
        previous_height = self.new_height
        previous_origin = self.origin
        previous_resolution = self.new_resolution

        msg = self.cached_msg
        original_width = msg.info.width
        original_height = msg.info.height
        original_resolution = msg.info.resolution
        original_origin = msg.info.origin
        original_data = msg.data

        self.new_width = original_width // self.downsample_factor
        self.new_height = original_height // self.downsample_factor
        self.new_resolution = original_resolution * self.downsample_factor
        if original_width % self.downsample_factor != 0:
            self.new_width += 1
        if original_height % self.downsample_factor != 0:
            self.new_height += 1

        self.low_res_grid = [0] * (self.new_width * self.new_height)
        for y in range(0, original_height, self.downsample_factor):
            for x in range(0, original_width, self.downsample_factor):
                new_x = x // self.downsample_factor
                new_y = y // self.downsample_factor
                new_idx = new_y * self.new_width + new_x
                cell_value = self.get_block_occupancy(
                    original_data, x, y, original_width, self.downsample_factor
                )
                self.low_res_grid[new_idx] = cell_value

        self.camera_seen_grid = self._merge_previous_seen_grid(
            previous_seen_grid,
            previous_width,
            previous_height,
            previous_origin,
            previous_resolution,
            original_origin,
        )

        self.robot_lowres_x, self.robot_lowres_y, self.trans = self.localization.get_robot_lowres_position(
            original_origin, original_resolution, self.downsample_factor
        )
        if self.robot_lowres_x is not None and self._is_cell_in_bounds(self.robot_lowres_x, self.robot_lowres_y):
            robot_idx = self.robot_lowres_y * self.new_width + self.robot_lowres_x
            self.low_res_grid[robot_idx] = 127
            self._mark_seen_cell(self.robot_lowres_x, self.robot_lowres_y, mark_robot=True)
            for objectlabel in self.inventory + ["{SELF}"]:
                self.previous_detections[objectlabel] = (
                    time.time(),
                    self.robot_lowres_x,
                    self.robot_lowres_y,
                    original_origin.position.x,
                    original_origin.position.y,
                    None,
                    None,
                    None,
                )
                self.previous_detections_truth[objectlabel] = (1.0, 0.9)
            self.node.get_logger().info(
                f"Marked robot position at ({self.robot_lowres_x}, {self.robot_lowres_y}) as occupied."
            )
        else:
            self.node.get_logger().warn("Robot position is out of bounds in the downsampled map.")

        depth_snapshot = self.object_detector.get_depth_snapshot()
        rgb_snapshot = self.object_detector.get_rgb_snapshot()
        depth_image = None if depth_snapshot is None else depth_snapshot.get("depth_image")
        object_detections = self.object_detector.detections
        detector_width = max(1, int(getattr(self.object_detector, "width", 1)))
        detector_height = max(1, int(getattr(self.object_detector, "height", 1)))
        depth_height = 0 if depth_image is None else depth_image.shape[0]
        depth_width = 0 if depth_image is None else depth_image.shape[1]

        if object_detections is not None:
            for out in object_detections:
                for detection in out:
                    scores = detection[5:]
                    if len(scores) == 0:
                        continue
                    class_id = int(np.argmax(scores))
                    if class_id >= len(self.object_detector.classes):
                        continue
                    confidence = self._sanitize_truth_confidence(scores[class_id])
                    if confidence <= self.object_detector.minconf:
                        continue
                    center_x = int(detection[0] * detector_width)
                    center_y = int(detection[1] * detector_height)
                    center_x = max(0, min(center_x, detector_width - 1))
                    center_y = max(0, min(center_y, detector_height - 1))
                    category_raw = self.object_detector.classes[class_id]
                    category = self._canonicalize_detection_category(
                        category_raw,
                        detection,
                        rgb_snapshot,
                        detector_width,
                        detector_height,
                    )
                    if depth_image is None:
                        self.node.get_logger().warn(f"Got detection ({category}) but no depth image")
                        continue
                    depth_x = max(0, min(center_x, depth_width - 1))
                    depth_y = max(0, min(center_y, depth_height - 1))
                    depth_value = float(depth_image[depth_y, depth_x])
                    if depth_value <= 0 or not math.isfinite(depth_value):
                        continue
                    if category not in self.M:
                        self.node.get_logger().info(
                            f"Detected object ({category}) does not have an assigned occupancy value in SemanticSLAM."
                        )
                        continue
                    self.node.get_logger().info(f"Detecting object ({category}) {depth_value}m away")
                    camera_point = PointStamped(
                        header=Header(stamp=Time().to_msg(), frame_id=self.camera_frame),
                        point=self._project_pixel_to_camera_point(
                            center_x, center_y, depth_value, width=detector_width, height=detector_height
                        ),
                    )
                    try:
                        camera_point.header.stamp = Time().to_msg()
                        transformed_point_map = self.tf_buffer.transform(
                            camera_point, self.map_frame, timeout=Duration(seconds=1.0)
                        )
                        camera_point.header.stamp = Time().to_msg()
                        transformed_point_base_link = self.tf_buffer.transform(
                            camera_point, self.base_frame, timeout=Duration(seconds=1.0)
                        )
                        object_grid_x, object_grid_y = self.get_lowres_position(
                            transformed_point_map.point.x,
                            transformed_point_map.point.y,
                            original_origin,
                            self.new_resolution,
                        )
                        if self._is_cell_in_bounds(object_grid_x, object_grid_y):
                            obj_idx = object_grid_y * self.new_width + object_grid_x
                            self.previous_detections[category] = (
                                time.time(),
                                object_grid_x,
                                object_grid_y,
                                original_origin.position.x,
                                original_origin.position.y,
                                transformed_point_map,
                                transformed_point_base_link,
                                (detection[0], detection[1], depth_value),
                            )
                            self.previous_detections_truth[category] = (1.0, confidence)
                            self.low_res_grid[obj_idx] = self.M[category]
                            self.node.get_logger().info(
                                f"Marked detected object ({category}) at ({object_grid_x}, {object_grid_y}) in grid."
                            )
                        else:
                            self.node.get_logger().warn(
                                f"Detected object ({category}) position is out of bounds in the downsampled map."
                            )
                    except Exception as exc:
                        self.node.get_logger().error(f"SemanticSLAM Transform exception: {str(exc)}")
        else:
            self.node.get_logger().warn("No detections received")

        for category in list(self.previous_detections.keys()):
            (
                t,
                object_grid_x,
                object_grid_y,
                old_origin_x,
                old_origin_y,
                old_point_map,
                old_point_base_link,
                imagecoords_depth,
            ) = self.previous_detections[category]
            current_origin_x = msg.info.origin.position.x
            current_origin_y = msg.info.origin.position.y
            offset_x = int(round((old_origin_x - current_origin_x) / self.new_resolution))
            offset_y = int(round((old_origin_y - current_origin_y) / self.new_resolution))
            new_object_grid_x = object_grid_x + offset_x
            new_object_grid_y = object_grid_y + offset_y
            self.previous_detections[category] = (
                t,
                new_object_grid_x,
                new_object_grid_y,
                current_origin_x,
                current_origin_y,
                old_point_map,
                old_point_base_link,
                imagecoords_depth,
            )
            if category in self.M and self._is_cell_in_bounds(new_object_grid_x, new_object_grid_y):
                obj_idx = new_object_grid_y * self.new_width + new_object_grid_x
                if (
                    time.time() - t < self.previous_detections_persistence
                    and self.low_res_grid[obj_idx] != 127
                ):
                    self.low_res_grid[obj_idx] = self.M[category]

        self._update_camera_seen_from_depth(original_origin, depth_snapshot)
        self.publish_low_res_map(msg)
        elapsed_time = time.time() - start_time
        self.node.get_logger().info(f"DONE map: {elapsed_time:.2f} seconds")
        import sys
        if self.robot_lowres_x is not None and any(arg.endswith(".metta") for arg in sys.argv):
            space_tick(self.node)

    def get_lowres_position(self, world_x, world_y, original_origin, original_resolution):
        grid_x = int((world_x - original_origin.position.x) / original_resolution)
        grid_y = int((world_y - original_origin.position.y) / original_resolution)
        return grid_x, grid_y

    def get_block_occupancy(self, data, x_start, y_start, width, factor):
        occupied = False
        empty = False
        for y in range(y_start, y_start + factor):
            for x in range(x_start, x_start + factor):
                if y < (len(data) // width) and x < width and (y * width + x) < len(data):
                    if data[y * width + x] == 100:
                        occupied = True
                    elif data[y * width + x] == 0:
                        empty = True
        if occupied:
            return 100
        if empty:
            return 0
        return -1

    def publish_low_res_map(self, original_msg):
        lowres_msg = OccupancyGrid()
        lowres_msg.header = original_msg.header
        lowres_msg.header.frame_id = self.map_frame
        lowres_msg.header.stamp = self.node.get_clock().now().to_msg()
        lowres_msg.info.resolution = self.new_resolution
        lowres_msg.info.width = self.new_width
        lowres_msg.info.height = self.new_height
        lowres_msg.info.origin = original_msg.info.origin
        lowres_msg.data = self.low_res_grid

        lowres_seen_msg = OccupancyGrid()
        lowres_seen_msg.header = original_msg.header
        lowres_seen_msg.header.frame_id = self.map_frame
        lowres_seen_msg.header.stamp = self.node.get_clock().now().to_msg()
        lowres_seen_msg.info.resolution = self.new_resolution
        lowres_seen_msg.info.width = self.new_width
        lowres_seen_msg.info.height = self.new_height
        lowres_seen_msg.info.origin = original_msg.info.origin
        lowres_seen_msg.data = self.camera_seen_grid if self.camera_seen_grid is not None else []

        self.origin = original_msg.info.origin
        self.mapupdate += 1
        if self.grid_dump_path:
            try:
                with open(self.grid_dump_path, "w") as f:
                    f.write(
                        str(self.new_width)
                        + "\n"
                        + str(self.new_height)
                        + "\n"
                        + str(self.robot_lowres_x)
                        + "\n"
                        + str(self.robot_lowres_y)
                        + "\n"
                        + str(self.low_res_grid)
                    )
            except Exception as exc:
                self.node.get_logger().error(f"Error writing grid file: {exc}")
        self.lowres_grid_pub.publish(lowres_msg)
        self.lowres_seen_grid_pub.publish(lowres_seen_msg)
