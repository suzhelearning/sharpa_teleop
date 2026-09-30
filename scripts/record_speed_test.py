#!/usr/bin/env python3
"""Record one Manus/Sharpa trajectory to SpeedTest.HDF5; never publish or enable hardware."""
import argparse
import math
from pathlib import Path
import time

import h5py
import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class Stream:
    """Append bounded batches; retain independent timestamps for each ROS topic."""

    def __init__(self, file, side, name, topic, shape, start_ns):
        self.group = file.create_group(f"{side}/{name}")
        self.group.attrs["topic"] = topic
        self.start_ns = start_ns
        self.rows = []
        self.count = 0
        self.shape = shape
        self.data = self.group.create_dataset(
            "values", shape=(0, *shape), maxshape=(None, *shape),
            chunks=(64, *shape), dtype="f8", compression="gzip", compression_opts=1,
        )
        self.times = {
            key: self.group.create_dataset(key, shape=(0,), maxshape=(None,), chunks=(64,), dtype="i8")
            for key in ("stamp_ns", "received_ros_ns", "elapsed_ns")
        }
        if len(shape) == 2:
            self.group.attrs["columns"] = "x,y,z,qx,qy,qz,qw"
            self.group.attrs["position_unit"] = "m"
        else:
            self.group.attrs["position_unit"] = "rad"

    def append(self, message, received_ros_ns, received_monotonic_ns):
        if isinstance(message, PoseArray):
            values = [(p.position.x, p.position.y, p.position.z,
                       p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                      for p in message.poses]
        else:
            values = message.position
            names = tuple(message.name)
            if len(names) != self.shape[0]:
                raise ValueError(f"{self.group.name}: expected {self.shape[0]} joint names")
            if "joint_names" not in self.group.attrs:
                self.group.attrs["joint_names"] = np.asarray(names, dtype=h5py.string_dtype())
            elif tuple(self.group.attrs["joint_names"]) != names:
                raise ValueError(f"{self.group.name}: joint order changed during recording")
        values = np.asarray(values, dtype=np.float64)
        if values.shape != self.shape:
            raise ValueError(f"{self.group.name}: expected {self.shape}, received {values.shape}")
        frame = message.header.frame_id
        if "frame_id" not in self.group.attrs:
            self.group.attrs["frame_id"] = frame
        elif self.group.attrs["frame_id"] != frame:
            raise ValueError(f"{self.group.name}: coordinate frame changed during recording")
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        self.rows.append((values, stamp, received_ros_ns, received_monotonic_ns - self.start_ns))
        if len(self.rows) >= 64:
            self.flush()

    def flush(self):
        if not self.rows:
            return
        end = self.count + len(self.rows)
        self.data.resize(end, axis=0)
        self.data[self.count:end] = np.stack([row[0] for row in self.rows])
        for column, dataset in enumerate(self.times.values(), start=1):
            dataset.resize(end, axis=0)
            dataset[self.count:end] = [row[column] for row in self.rows]
        self.count = end
        self.rows.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("SpeedTest.HDF5"),
                        help="new recording file; existing files are never overwritten")
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="recording duration; default 0 records until Ctrl+C")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and non-negative")
    if args.output.exists():
        parser.error(f"{args.output} already exists; move it aside or choose --output")

    with h5py.File(args.output, "x") as file:
        file.attrs["format"] = "sharpa_speed_test"
        file.attrs["version"] = 1
        file.attrs["complete"] = False
        rclpy.init()
        node = Node("speed_test_recorder")
        start_ns = time.monotonic_ns()
        file.attrs["start_ros_ns"] = node.get_clock().now().nanoseconds
        file.attrs["ros_domain_id"] = node.context.get_domain_id()
        streams = []
        subscriptions = []
        completed = False
        try:
            for side in ("left", "right"):
                for name, topic, kind, shape in (
                    ("manus_raw", f"/manus/{side}/raw_poses", PoseArray, (25, 7)),
                    ("sharpa_joint", f"/sharpa/{side}/command", JointState, (22,)),
                ):
                    stream = Stream(file, side, name, topic, shape, start_ns)
                    streams.append(stream)
                    def receive(message, stream=stream):
                        received_ns = time.monotonic_ns()
                        stream.append(message, node.get_clock().now().nanoseconds, received_ns)
                    subscriptions.append(node.create_subscription(kind, topic, receive, qos_profile_sensor_data))
            print(f"Recording {args.output.resolve()} (ROS domain {node.context.get_domain_id()}). "
                  "Ctrl+C saves and stops. No hardware control.", flush=True)
            try:
                while rclpy.ok() and (args.seconds == 0 or time.monotonic_ns() - start_ns < args.seconds * 1e9):
                    rclpy.spin_once(node, timeout_sec=0.1)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            completed = True
        finally:
            for stream in streams:
                stream.flush()
            file.attrs["complete"] = completed
            file.flush()
            node.destroy_node()
            rclpy.try_shutdown()
        for stream in streams:
            print(f"{stream.group.name}: {stream.count} frames")
        if any(stream.count == 0 for stream in streams):
            print("WARNING: some streams are empty; check that pixi run manus is running in the same ROS domain.")
        print(f"Saved {args.output.resolve()}")

if __name__ == "__main__":
    main()