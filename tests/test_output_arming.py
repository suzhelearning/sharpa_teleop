import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sharpa_teleop'))
from sharpa_teleop.sharpa_output import SharpaOutput


class SlowBackend:
    def __init__(self, *args):
        self.started = threading.Event()
        self.disabled = threading.Event()
        self.enabled = False

    def arm(self, targets):
        self.started.set()
        time.sleep(0.8)
        self.enabled = True
        return {'left': (0.0,) * 22}

    def disable_all(self):
        self.enabled = False
        self.disabled.set()

    def set_positions(self, side, positions):
        pass

    def close(self):
        self.disable_all()


class ArmingTests(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        root = Path(directory.name)
        for side in ('left', 'right'):
            urdf = root / 'urdf' / f'{side}_sharpa_wave' / f'{side}_sharpa_wave.urdf'
            urdf.parent.mkdir(parents=True)
            joints = ''.join(
                f'<joint name="{side}_joint_{i}" type="revolute">'
                '<limit lower="-1" upper="1"/></joint>' for i in range(22)
            )
            urdf.write_text(f'<robot name="test">{joints}</robot>')
        rclpy.init(domain_id=80, args=[
            '--ros-args', '-p', f'sdk_root:={root}', '-p', 'dry_run:=false',
            '-p', 'left_serial:=TEST_LEFT', '-p', 'feedback_hz:=0.0',
        ])
        self.addCleanup(rclpy.shutdown)
        self.backend = SlowBackend()
        with patch('sharpa_teleop.sharpa_output.NativeSharpaBackend', return_value=self.backend):
            self.output = SharpaOutput()
        self.addCleanup(self.output.destroy_node)
        self.probe = Node('arming_probe')
        self.addCleanup(self.probe.destroy_node)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.output)
        self.executor.add_node(self.probe)
        self.publisher = self.probe.create_publisher(
            JointState, '/sharpa/left/command', qos_profile_sensor_data)
        self.received = threading.Event()
        self.subscription = self.probe.create_subscription(
            JointState, '/sharpa/left/target', lambda msg: self.received.set(), qos_profile_sensor_data)
        self.client = self.probe.create_client(SetBool, '/sharpa/enable')
        self.thread = threading.Thread(target=self.executor.spin)
        self.thread.start()
        self.addCleanup(self.stop_executor)
        self.assertTrue(self.client.wait_for_service(timeout_sec=2))

    def stop_executor(self):
        self.executor.shutdown()
        self.thread.join(timeout=2)

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.probe.get_clock().now().to_msg()
        msg.name = [f'left_joint_{i}' for i in range(22)]
        msg.position = [0.0] * 22
        self.publisher.publish(msg)

    def request_arm(self):
        deadline = time.monotonic() + 2
        while not self.received.is_set() and time.monotonic() < deadline:
            self.publish()
            self.received.wait(0.02)
        self.assertTrue(self.received.is_set(), 'valid target was not received')
        future = self.client.call_async(SetBool.Request(data=True))
        self.assertTrue(self.backend.started.wait(1), 'arming did not start')
        return future

    def finish(self, future):
        deadline = time.monotonic() + 3
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        self.assertTrue(future.done(), 'arming service did not finish')
        return future.result()

    def test_input_loss_during_slow_arm_uses_last_valid_target(self):
        response = self.finish(self.request_arm())
        self.assertTrue(response.success, response.message)
        self.assertTrue(self.backend.enabled)
        self.assertFalse(self.backend.disabled.is_set())


if __name__ == '__main__':
    unittest.main()
