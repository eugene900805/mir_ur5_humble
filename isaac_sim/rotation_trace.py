#!/usr/bin/env python3
"""Trace RotationShim's sampled path heading against the localized robot yaw."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RotationTrace(Node):
    def __init__(self, duration):
        super().__init__("rotation_trace")
        self.deadline = time.monotonic() + duration
        self.started = time.monotonic()
        self.pose = None
        self.raw = (0.0, 0.0)
        self.smooth = (0.0, 0.0)
        self.odom = (0.0, 0.0)
        self.create_subscription(Path, "/plan", self._plan, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose, 20)
        self.create_subscription(Twist, "/cmd_vel_nav", self._raw, 20)
        self.create_subscription(
            Twist, "/diff_cont/cmd_vel_unstamped", self._smooth, 20)
        self.create_subscription(Odometry, "/odom", self._odom, 20)

    def _pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw(p.orientation))

    def _raw(self, msg):
        self.raw = (msg.linear.x, msg.angular.z)

    def _smooth(self, msg):
        self.smooth = (msg.linear.x, msg.angular.z)

    def _odom(self, msg):
        twist = msg.twist.twist
        self.odom = (math.hypot(twist.linear.x, twist.linear.y), twist.angular.z)

    def _plan(self, msg):
        if self.pose is None or len(msg.poses) < 2:
            return
        start = msg.poses[0].pose.position
        sample = None
        sample_index = 0
        for index, stamped in enumerate(msg.poses[1:], 1):
            point = stamped.pose.position
            if math.hypot(point.x - start.x, point.y - start.y) >= 0.5:
                sample = point
                sample_index = index
                break
        if sample is None:
            return
        desired = math.atan2(sample.y - start.y, sample.x - start.x)
        error = wrap(desired - self.pose[2])
        elapsed = time.monotonic() - self.started
        print(
            f"t={elapsed:5.1f} pose=({self.pose[0]:.2f},{self.pose[1]:.2f},"
            f"{self.pose[2]:+.2f}) path_yaw={desired:+.2f} err={error:+.2f} "
            f"idx={sample_index} raw=({self.raw[0]:+.2f},{self.raw[1]:+.2f}) "
            f"smooth=({self.smooth[0]:+.2f},{self.smooth[1]:+.2f}) "
            f"odom=({self.odom[0]:+.2f},{self.odom[1]:+.2f})",
            flush=True,
        )


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    rclpy.init()
    node = RotationTrace(duration)
    while rclpy.ok() and time.monotonic() < node.deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
