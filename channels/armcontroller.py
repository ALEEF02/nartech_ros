import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String


def _get_or_declare_parameter(node, name, default):
    if node.has_parameter(name):
        return node.get_parameter(name).value
    return node.declare_parameter(name, default).value


# Visual servoing toward the detected orange ball.
GOAL_TARGET_DISTANCE = 0.44
YAW_TOL = 0.05
DEPTH_TOL = 0.03
K_YAW = 0.3
K_FWD = 0.25
CORRECTION_DT = 0.5
BACKUP_SPEED = -0.08
BACKUP_LOSSTIME = 5.0

# Right arm pose targets, expressed in the MuJoCo ROS bridge base_link frame.
PREGRASP_BACKOFF_X = 0.06
PREGRASP_LIFT_Z = 0.07
GRASP_FORWARD_OFFSET_X = -0.015
GRASP_LIFT_OFFSET_Z = 0.01
POST_GRASP_LIFT_Z = 0.10
CARRY_POSE = (0.18, -0.18, 0.30)
DROP_POSE = (0.24, -0.16, 0.18)
STOW_POSE = (0.16, -0.22, 0.26)

ARM_GOAL_TIMEOUT_SEC = 6.0
HAND_TIMEOUT_SEC = 3.0
GRASP_TIMEOUT_SEC = 2.0
DROP_CLEAR_TIMEOUT_SEC = 2.0
GRASP_LOSS_TIMEOUT_SEC = 0.5

STATUS_IDLE = "IDLE"
STATUS_BUSY = "BUSY"
STATUS_SUCCESS = "SUCCESS"
STATUS_FAIL = "FAIL"

HAND_OPEN = "OPEN"
HAND_CLOSED = "CLOSED"
HAND_MOVING = "MOVING"

RIGHT_ARM_GOAL_TOPIC = "/unitree/right_arm/goal_pose"
RIGHT_ARM_STATUS_TOPIC = "/unitree/right_arm/status"
RIGHT_HAND_COMMAND_TOPIC = "/unitree/right_hand/command"
RIGHT_HAND_STATE_TOPIC = "/unitree/right_hand/state"
GRASPED_OBJECT_LABEL_TOPIC = "/unitree/grasped_object_label"


# Assume mettabridge defines these constants and functions:
if __name__ == "__main__":
    NAV_STATE_BUSY = NAV_STATE_SUCCESS = NAV_STATE_FAIL = 42
    NAV_STATE_SET = lambda x: 42
    NAV_STATE_GET = lambda: 42
    ARM_STATE_SET = lambda x: 42
    ARM_STATE_GET = lambda: 42
else:
    from mettabridge import (
        ARM_STATE_GET,
        ARM_STATE_SET,
        NAV_STATE_BUSY,
        NAV_STATE_FAIL,
        NAV_STATE_GET,
        NAV_STATE_SET,
        NAV_STATE_SUCCESS,
    )


class ArmController:
    def __init__(self, node=None, semantic_slam=None, navigation=None):
        ARM_STATE_SET("FREE")
        self.available = True
        self.semantic_slam = semantic_slam
        self.navigation = navigation
        self.objectlabel = None
        if node is None:
            self.own_node = Node("arm_controller")
            self.node = self.own_node
        else:
            self.own_node = None
            self.node = node

        self.base_frame = _get_or_declare_parameter(self.node, "base_frame", "base_link")
        self.picking = False
        self.dropping = False
        self.cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.goal_pose_pub = self.node.create_publisher(PoseStamped, RIGHT_ARM_GOAL_TOPIC, 10)
        self.hand_command_pub = self.node.create_publisher(String, RIGHT_HAND_COMMAND_TOPIC, 10)

        self.right_arm_status = STATUS_IDLE
        self.right_hand_state = HAND_OPEN
        self.grasped_object_label = ""
        self._grasp_missing_since = None
        self._owned_timers = []

        self.node.create_subscription(String, RIGHT_ARM_STATUS_TOPIC, self._right_arm_status_cb, 10)
        self.node.create_subscription(String, RIGHT_HAND_STATE_TOPIC, self._right_hand_state_cb, 10)
        self.node.create_subscription(String, GRASPED_OBJECT_LABEL_TOPIC, self._grasped_object_label_cb, 10)
        self._grasp_watchdog = self.node.create_timer(0.2, self._check_grasp_state)
        self._publish_hand_command("open")
        self._publish_arm_goal(*STOW_POSE)

    def _normalize_upper_state(self, value, default):
        text = str(value).strip().upper()
        return text if text else default

    def _right_arm_status_cb(self, msg: String):
        self.right_arm_status = self._normalize_upper_state(msg.data, STATUS_IDLE)

    def _right_hand_state_cb(self, msg: String):
        self.right_hand_state = self._normalize_upper_state(msg.data, HAND_OPEN)

    def _grasped_object_label_cb(self, msg: String):
        self.grasped_object_label = str(msg.data).strip()
        if self.grasped_object_label:
            self._grasp_missing_since = None

    def _check_grasp_state(self):
        carrying = ARM_STATE_GET()
        if carrying == "FREE":
            self._grasp_missing_since = None
            return
        if self.picking or self.dropping:
            self._grasp_missing_since = None
            return
        if self.grasped_object_label == carrying:
            self._grasp_missing_since = None
            return
        if self._grasp_missing_since is None:
            self._grasp_missing_since = time.time()
            return
        if time.time() - self._grasp_missing_since < GRASP_LOSS_TIMEOUT_SEC:
            return
        self.node.get_logger().warn(
            f"arm_controller: Lost grasp of {carrying}; clearing manipulation state"
        )
        ARM_STATE_SET("FREE")
        NAV_STATE_SET(NAV_STATE_FAIL)
        self._grasp_missing_since = None

    def _destroy_owned_timer(self, timer):
        if timer is None:
            return
        try:
            timer.cancel()
        except Exception:
            pass
        try:
            self.node.destroy_timer(timer)
        except Exception:
            pass
        if timer in self._owned_timers:
            self._owned_timers.remove(timer)

    def _wait_for_predicate(self, predicate, timeout_sec, description) -> Future:
        future = Future()
        start_time = time.time()
        holder = {"timer": None}

        def _poll():
            if future.done():
                self._destroy_owned_timer(holder["timer"])
                return
            if predicate():
                future.set_result(True)
                self._destroy_owned_timer(holder["timer"])
                return
            if time.time() - start_time >= timeout_sec:
                self.node.get_logger().warn(f"arm_controller: Timeout waiting for {description}")
                future.set_result(False)
                self._destroy_owned_timer(holder["timer"])

        holder["timer"] = self.node.create_timer(0.05, _poll)
        self._owned_timers.append(holder["timer"])
        return future

    def _publish_hand_command(self, command: str):
        msg = String()
        msg.data = command
        self.hand_command_pub.publish(msg)
        self.right_hand_state = HAND_MOVING
        self.node.get_logger().info(f"arm_controller: hand command -> {command}")

    def _publish_arm_goal(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.right_arm_status = STATUS_BUSY
        self.goal_pose_pub.publish(msg)
        self.node.get_logger().info(
            f"arm_controller: arm goal -> x={x:.3f} y={y:.3f} z={z:.3f}"
        )

    def _wait_for_hand_state(self, desired_state: str, timeout_sec: float) -> Future:
        return self._wait_for_predicate(
            lambda: self.right_hand_state == desired_state,
            timeout_sec,
            f"hand state {desired_state}",
        )

    def _wait_for_arm_result(self, timeout_sec: float) -> Future:
        return self._wait_for_predicate(
            lambda: self.right_arm_status in (STATUS_SUCCESS, STATUS_FAIL),
            timeout_sec,
            "arm motion result",
        )

    def _wait_for_grasp_label(self, label: str, timeout_sec: float) -> Future:
        return self._wait_for_predicate(
            lambda: self.grasped_object_label == label,
            timeout_sec,
            f"grasp label {label}",
        )

    def _wait_for_label_clear(self, timeout_sec: float) -> Future:
        return self._wait_for_predicate(
            lambda: self.grasped_object_label == "",
            timeout_sec,
            "grasp label clear",
        )

    def _arm_goal_and_wait(self, pose, timeout_sec: float, next_cb):
        self._publish_arm_goal(*pose)

        def _after_wait(fut):
            ok = bool(fut.result())
            if not ok or self.right_arm_status != STATUS_SUCCESS:
                next_cb(False)
                return
            next_cb(True)

        self._wait_for_arm_result(timeout_sec).add_done_callback(_after_wait)

    def _hand_command_and_wait(self, command: str, desired_state: str, timeout_sec: float, next_cb):
        self._publish_hand_command(command)

        def _after_wait(fut):
            next_cb(bool(fut.result()))

        self._wait_for_hand_state(desired_state, timeout_sec).add_done_callback(_after_wait)

    def _stop_motion(self):
        self.cmd_pub.publish(Twist())

    def _make_pregrasp_pose(self, target_x, target_y, target_z):
        return (
            max(0.14, target_x - PREGRASP_BACKOFF_X),
            target_y,
            max(0.04, target_z + PREGRASP_LIFT_Z),
        )

    def _make_grasp_pose(self, target_x, target_y, target_z):
        return (
            max(0.12, target_x + GRASP_FORWARD_OFFSET_X),
            target_y,
            max(0.02, target_z + GRASP_LIFT_OFFSET_Z),
        )

    def _make_lift_pose(self, target_x, target_y, target_z):
        return (
            max(CARRY_POSE[0], target_x - 0.02),
            target_y,
            max(CARRY_POSE[2], target_z + POST_GRASP_LIFT_Z),
        )

    def pick_at(self, x: float, y: float, z: float, objectlabel: str, *, done_cb=None) -> Future:
        result_future = Future()
        pregrasp_pose = self._make_pregrasp_pose(x, y, z)
        grasp_pose = self._make_grasp_pose(x, y, z)
        lift_pose = self._make_lift_pose(x, y, z)

        def _finish(success: bool):
            if done_cb is not None:
                try:
                    done_cb(success)
                except Exception as exc:
                    self.node.get_logger().warn(f"arm_controller: done_cb raised {exc!r}")
            if not result_future.done():
                result_future.set_result(success)

        def _abort(reason: str):
            self.node.get_logger().error(f"arm_controller: ABORT - {reason}")
            self._finish_pick_failure()
            _finish(False)

        def _after_open(ok: bool):
            if not ok:
                _abort("hand failed to open")
                return
            self._arm_goal_and_wait(pregrasp_pose, ARM_GOAL_TIMEOUT_SEC, _after_pregrasp)

        def _after_pregrasp(ok: bool):
            if not ok:
                _abort("pregrasp motion failed")
                return
            self._arm_goal_and_wait(grasp_pose, ARM_GOAL_TIMEOUT_SEC, _after_grasp_pose)

        def _after_grasp_pose(ok: bool):
            if not ok:
                _abort("grasp motion failed")
                return
            self._hand_command_and_wait("close", HAND_CLOSED, HAND_TIMEOUT_SEC, _after_close)

        def _after_close(ok: bool):
            if not ok:
                _abort("hand failed to close")
                return

            def _after_label(label_fut):
                if not bool(label_fut.result()):
                    _abort(f"failed to physically grasp {objectlabel}")
                    return
                self._arm_goal_and_wait(lift_pose, ARM_GOAL_TIMEOUT_SEC, _after_lift)

            self._wait_for_grasp_label(objectlabel, GRASP_TIMEOUT_SEC).add_done_callback(_after_label)

        def _after_lift(ok: bool):
            if not ok:
                _abort("lift motion failed")
                return
            self._arm_goal_and_wait(CARRY_POSE, ARM_GOAL_TIMEOUT_SEC, _after_carry)

        def _after_carry(ok: bool):
            if not ok:
                _abort("carry pose failed")
                return
            self.node.get_logger().info("arm_controller: PICK complete")
            _finish(True)

        self._hand_command_and_wait("open", HAND_OPEN, HAND_TIMEOUT_SEC, _after_open)
        return result_future

    def _finish_pick_failure(self):
        self.picking = False
        ARM_STATE_SET("FREE")
        NAV_STATE_SET(NAV_STATE_FAIL)
        self._publish_hand_command("open")
        self._publish_arm_goal(*STOW_POSE)

    def pick(self, objectlabel: str, recover=False) -> None:
        self.objectlabel = objectlabel
        if not recover:
            if NAV_STATE_GET() == NAV_STATE_BUSY or self.picking or ARM_STATE_GET() != "FREE":
                NAV_STATE_SET(NAV_STATE_FAIL)
                self.node.get_logger().info(
                    "arm_controller: Pick skipped, already in progress or arm not free"
                )
                return
        else:
            if self.picking:
                self.node.get_logger().warn("arm_controller: Recovering, but pick already in progress")
                NAV_STATE_SET(NAV_STATE_FAIL)
                return

        NAV_STATE_SET(NAV_STATE_BUSY)
        self.picking = True
        if not objectlabel or objectlabel not in self.semantic_slam.previous_detections:
            self.node.get_logger().info(
                "arm_controller: Pick failed, object location not observed or remembered"
            )
            ARM_STATE_SET("FREE")
            NAV_STATE_SET(NAV_STATE_FAIL)
            self.picking = False
            return

        self.correction_attempts = 0
        self.max_corrections = 40

        def yaw_from_quat(q):
            s = 2.0 * (q.w * q.z + q.x * q.y)
            c = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(s, c)

        def correction_step():
            (
                t,
                _,
                _,
                _,
                _,
                spoint_map,
                spoint_base_link,
                imagecoords_depth,
            ) = self.semantic_slam.previous_detections[objectlabel]

            if time.time() - t > BACKUP_LOSSTIME or spoint_base_link is None or imagecoords_depth is None:
                self.correction_attempts += 1
                back_twist = Twist()
                back_twist.linear.x = BACKUP_SPEED
                try:
                    tf = self.semantic_slam.trans
                    rx = tf.transform.translation.x
                    ry = tf.transform.translation.y
                    yaw = yaw_from_quat(tf.transform.rotation)
                    dx = spoint_map.point.x - rx
                    dy = spoint_map.point.y - ry
                    angle = math.atan2(dy, dx) - yaw
                    angle = (angle + math.pi) % (2 * math.pi) - math.pi
                    back_twist.angular.z = K_YAW * angle
                except Exception as exc:
                    self.node.get_logger().warn(f"arm_controller: backing away failed: {exc}")
                self.cmd_pub.publish(back_twist)
                if self.correction_attempts >= self.max_corrections:
                    self._stop_motion()
                    self._destroy_owned_timer(self.correction_timer)
                    self._finish_pick_failure()
                return

            target_point = spoint_base_link.point
            x_rel, _, depth = imagecoords_depth
            x_err = (x_rel - 0.5) * 2.0
            depth_err = depth - GOAL_TARGET_DISTANCE
            twist = Twist()
            if abs(x_err) > YAW_TOL:
                twist.angular.z = -K_YAW * x_err
            if abs(depth_err) > DEPTH_TOL:
                twist.linear.x = K_FWD * depth_err
            self.cmd_pub.publish(twist)
            self.node.get_logger().info(
                f"arm_controller: align x_err={x_err:+.2f} depth={depth:.2f} "
                f"cmd=({twist.linear.x:.2f}, {twist.angular.z:.2f})"
            )

            if abs(x_err) <= YAW_TOL and abs(depth_err) <= DEPTH_TOL:
                self._stop_motion()
                self._destroy_owned_timer(self.correction_timer)
                self.node.get_logger().info("arm_controller: alignment complete; executing pick")

                def _on_pick_complete(success: bool):
                    if success:
                        ARM_STATE_SET(objectlabel)
                        NAV_STATE_SET(NAV_STATE_SUCCESS)
                    else:
                        ARM_STATE_SET("FREE")
                        NAV_STATE_SET(NAV_STATE_FAIL)
                    self.picking = False

                self.pick_at(
                    target_point.x,
                    target_point.y,
                    target_point.z,
                    objectlabel,
                    done_cb=_on_pick_complete,
                )
                return

            self.correction_attempts += 1
            if self.correction_attempts >= self.max_corrections:
                self._stop_motion()
                self._destroy_owned_timer(self.correction_timer)
                self._finish_pick_failure()

        self.correction_timer = self.node.create_timer(CORRECTION_DT, correction_step)
        self._owned_timers.append(self.correction_timer)

    def drop(self):
        if NAV_STATE_GET() == NAV_STATE_BUSY or ARM_STATE_GET() == "FREE" or self.dropping:
            return

        held_label = ARM_STATE_GET()
        NAV_STATE_SET(NAV_STATE_BUSY)
        self.dropping = True

        def _finish(success: bool):
            self.dropping = False
            ARM_STATE_SET("FREE" if success else held_label)
            NAV_STATE_SET(NAV_STATE_SUCCESS if success else NAV_STATE_FAIL)

        def _after_stow(ok: bool):
            if not ok:
                _finish(False)
                return
            self.node.get_logger().info("arm_controller: DROP complete")
            _finish(True)

        def _after_clear(clear_fut):
            if not bool(clear_fut.result()):
                _finish(False)
                return
            self._arm_goal_and_wait(STOW_POSE, ARM_GOAL_TIMEOUT_SEC, _after_stow)

        def _after_open(ok: bool):
            if not ok:
                _finish(False)
                return
            self._wait_for_label_clear(DROP_CLEAR_TIMEOUT_SEC).add_done_callback(_after_clear)

        def _after_drop_pose(ok: bool):
            if not ok:
                _finish(False)
                return
            self._hand_command_and_wait("open", HAND_OPEN, HAND_TIMEOUT_SEC, _after_open)

        self._arm_goal_and_wait(DROP_POSE, ARM_GOAL_TIMEOUT_SEC, _after_drop_pose)


def main():
    rclpy.init()
    armcontroller = ArmController()
    armcontroller._publish_hand_command("open")
    armcontroller._publish_arm_goal(*STOW_POSE)
    end_time = time.time() + 5.0
    while time.time() < end_time and rclpy.ok():
        rclpy.spin_once(armcontroller.node, timeout_sec=0.1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
