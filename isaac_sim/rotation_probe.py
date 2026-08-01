#!/usr/bin/env python3
"""Measure the rotate-to-translate handoff on the live Nav2 command chain."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class RotationProbe(Node):
    def __init__(self, duration):
        super().__init__("rotation_probe")
        self.deadline = time.monotonic() + duration
        self.samples = {"raw": [], "smooth": [], "odom": []}
        self.create_subscription(Twist, "/cmd_vel_nav", self._raw, 50)
        self.create_subscription(
            Twist, "/diff_cont/cmd_vel_unstamped", self._smooth, 50)
        self.create_subscription(Odometry, "/odom", self._odom, 50)

    def _append(self, channel, linear, angular):
        self.samples[channel].append((time.monotonic(), linear, angular))

    def _raw(self, msg):
        self._append("raw", msg.linear.x, msg.angular.z)

    def _smooth(self, msg):
        self._append("smooth", msg.linear.x, msg.angular.z)

    def _odom(self, msg):
        linear = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self._append("odom", linear, msg.twist.twist.angular.z)

    def done(self):
        return time.monotonic() >= self.deadline


def summarize(name, samples):
    if not samples:
        print(f"{name}: no samples")
        return
    active = [sample for sample in samples if abs(sample[1]) > 0.01 or abs(sample[2]) > 0.03]
    rotate = [sample for sample in active if abs(sample[1]) <= 0.01 and abs(sample[2]) > 0.03]
    if not rotate:
        print(f"{name}: no rotate-only phase ({len(samples)} samples)")
        return
    rotate_start = rotate[0][0]
    forward = next(
        (sample for sample in samples if sample[0] >= rotate_start and abs(sample[1]) > 0.02),
        None,
    )
    before_forward = [sample for sample in rotate if forward is None or sample[0] < forward[0]]
    rotate_end = before_forward[-1][0] if before_forward else rotate_start
    gaps = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
    peak_w = max(abs(sample[2]) for sample in rotate)
    if forward is None:
        print(f"{name}: rotate started; no forward phase; peak |w|={peak_w:.3f}")
        return
    print(
        f"{name}: rotate={rotate_end - rotate_start:.3f}s wall, "
        f"handoff pause={forward[0] - rotate_end:.3f}s, "
        f"first vx={forward[1]:.3f}, peak |w|={peak_w:.3f}, "
        f"max message gap={max(gaps, default=0.0):.3f}s"
    )


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init()
    node = RotationProbe(duration)
    while rclpy.ok() and not node.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    for channel in ("raw", "smooth", "odom"):
        summarize(channel, node.samples[channel])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
