"""MuJoCo finger dynamics driven by ROS targets; never imports the hardware SDK."""
import math
import os
import signal
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from .safety import header_stamp_nanoseconds, is_fresh, validate_command
from .sim_model import HandSimulation, SIDES


class MujocoSimulation(Node):
    def __init__(self):
        super().__init__("mujoco_sim")
        models_root = self.declare_parameter("models_root", os.environ.get("SHARPA_MODELS", "")).value
        self.headless = self.declare_parameter("headless", False).value
        self.timeout = float(self.declare_parameter("timeout_sec", 0.5).value)
        feedback_hz = float(self.declare_parameter("feedback_hz", 30.0).value)
        if not isinstance(self.headless, bool):
            raise ValueError("headless must be a boolean")
        if any(not math.isfinite(value) or value <= 0 for value in (self.timeout, feedback_hz)):
            raise ValueError("timeout_sec and feedback_hz must be finite and positive")
        self.simulation = HandSimulation(models_root)
        self._last_stamp = {side: 0 for side in SIDES}
        self._last_receipt = {side: None for side in SIDES}
        self._tracking = {side: False for side in SIDES}
        self._viewer = None
        self.closed = False
        self._last_warning = {side: 0.0 for side in SIDES}
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._feedback = {side: self.create_publisher(
            JointState, f"/sim/sharpa/{side}/joint_states", qos) for side in SIDES}
        self._commands = [self.create_subscription(
            JointState, f"/sharpa/{side}/command",
            lambda msg, side=side: self._on_command(side, msg), qos) for side in SIDES]
        self._step_timer = self.create_timer(self.simulation.model.opt.timestep, self._step)
        self._feedback_timer = self.create_timer(1.0 / feedback_hz, self._publish_feedback)
        self._viewer_timer = None
        if not self.headless:
            import mujoco.viewer
            self._viewer = mujoco.viewer.launch_passive(
                self.simulation.model, self.simulation.data,
                show_left_ui=False, show_right_ui=False)
            self.simulation.configure_camera(self._viewer.cam)
            self._viewer_timer = self.create_timer(1.0 / 30.0, self._sync_viewer)
        self.get_logger().info(
            f"MuJoCo ready: 44 finger joints, fixed wrists, headless={self.headless}; "
            "feedback=/sim/sharpa/{left,right}/joint_states; hardware SDK is not loaded by this node")

    def _on_command(self, side, message):
        now = time.monotonic()
        try:
            stamp = header_stamp_nanoseconds(message.header.stamp)
            checked = validate_command(
                self.simulation.joints[side], message.name, message.position,
                stamp, self._last_stamp[side], self.get_clock().now().nanoseconds,
                self.timeout, 0.0)
            if not checked.accepted:
                raise ValueError(checked.reason)
            self.simulation.command(side, message.name, checked.positions)
        except ValueError as error:
            if now - self._last_warning[side] >= 1.0:
                self.get_logger().warning(f"Rejected {side} simulation command: {error}")
                self._last_warning[side] = now
            return
        self._last_stamp[side] = stamp
        self._last_receipt[side] = now
        self._tracking[side] = True

    def _step(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        for side in SIDES:
            source_age = (ros_now - self._last_stamp[side]) / 1e9
            if self._tracking[side] and (
                not is_fresh(self._last_receipt[side], now, self.timeout)
                or source_age > self.timeout or source_age < 0
            ):
                self.simulation.hold(side)
                self._tracking[side] = False
                self.get_logger().warning(f"{side} input expired; holding current simulated pose")
        self.simulation.step()

    def _publish_feedback(self):
        stamp = self.get_clock().now().to_msg()
        for side in SIDES:
            message = JointState()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.name = list(self.simulation.joints[side].names)
            message.position, message.velocity = self.simulation.feedback(side)
            self._feedback[side].publish(message)

    def _sync_viewer(self):
        if not self._viewer.is_running():
            self.closed = True
        else:
            self._viewer.sync()

    def destroy_node(self):
        if self._viewer is not None:
            self._viewer.close()
            self._viewer = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MujocoSimulation()
        while rclpy.ok() and not node.closed:
            rclpy.spin_once(node, timeout_sec=0.02)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
