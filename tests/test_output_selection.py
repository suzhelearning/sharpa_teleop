import sys
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/sharpa_teleop'))
from sharpa_teleop.sharpa_output import SharpaOutput, SdkFailure


class OutputSelectionTests(unittest.TestCase):
    def test_left_only_output_ignores_right_and_clips_left_limit_violations(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for side in ('left', 'right'):
                urdf = root / 'urdf' / f'{side}_sharpa_wave' / f'{side}_sharpa_wave.urdf'
                urdf.parent.mkdir(parents=True)
                joints = ''.join(
                    f'<joint name="{side}_joint_{i}" type="revolute">'
                    '<limit lower="-0.17453293" upper="1.5708"/></joint>'
                    for i in range(22)
                )
                urdf.write_text(f'<robot name="test">{joints}</robot>')
            rclpy.init(domain_id=79, args=[
                '--ros-args', '-p', f'sdk_root:={root}',
                '-p', 'dry_run:=false', '-p', 'left_serial:=TEST_LEFT',
            ])
            self.addCleanup(rclpy.shutdown)
            # Exercise the user's fail-closed startup without loading or contacting hardware.
            with patch('sharpa_teleop.sharpa_output.NativeSharpaBackend',
                       side_effect=SdkFailure('device unavailable in test')):
                output = SharpaOutput()
            self.addCleanup(output.destroy_node)
            probe = Node('selection_probe')
            self.addCleanup(probe.destroy_node)
            executor = SingleThreadedExecutor()
            executor.add_node(output)
            executor.add_node(probe)
            self.addCleanup(executor.shutdown)
            received = {'left': [], 'right': []}
            subscriptions = [
                probe.create_subscription(
                    JointState, f'/sharpa/{side}/target',
                    lambda msg, side=side: received[side].append(msg), qos_profile_sensor_data,
                ) for side in received
            ]
            publishers = {
                side: probe.create_publisher(JointState, f'/sharpa/{side}/command', qos_profile_sensor_data)
                for side in received
            }

            def spin(seconds):
                deadline = time.monotonic() + seconds
                while time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=0.01)

            def publish(side, position):
                msg = JointState()
                msg.header.stamp = probe.get_clock().now().to_msg()
                msg.name = [f'{side}_joint_{i}' for i in range(22)]
                msg.position = [position] * 22
                publishers[side].publish(msg)

            spin(0.3)
            for _ in range(5):
                publish('left', 0.2)
                publish('right', 0.3)
                spin(0.05)
            spin(0.1)
            self.assertTrue(received['left'], 'selected left commands must reach the target topic')
            self.assertEqual(list(received['left'][-1].position), [0.2] * 22)
            self.assertEqual(received['right'], [], 'unselected right commands must not become targets')
            publish('left', -0.217018941)
            spin(0.2)
            self.assertEqual(list(received['left'][-1].position), [-0.17453293] * 22)


if __name__ == '__main__':
    unittest.main()
