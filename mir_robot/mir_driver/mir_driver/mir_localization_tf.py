#!/usr/bin/env python3
# Broadcast map->odom from the MiR's onboard localization.
#
# The bridge relays the robot's /robot_pose (base_footprint in the robot's
# map frame, ~10 Hz) but its /tf relay does not contain map->odom. This node
# closes that gap: map->odom = T_map_base * inverse(T_odom_base), so RViz and
# Nav2 use exactly the pose shown in the MiR web interface — no local AMCL,
# no 2D Pose Estimate needed.

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


def q_mult(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_inv(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    return (-x / n, -y / n, -z / n, w / n)


def q_rotate(q, v):
    # rotate vector v by quaternion q
    qv = (v[0], v[1], v[2], 0.0)
    rx, ry, rz, _ = q_mult(q_mult(q, qv), q_inv(q))
    return (rx, ry, rz)


class MirLocalizationTf(Node):
    def __init__(self):
        super().__init__('mir_localization_tf')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.warned = False

        self.create_subscription(Pose, 'robot_pose', self.robot_pose_cb, 10)

    def robot_pose_cb(self, pose):
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time())
        except Exception as e:
            if not self.warned:
                self.get_logger().warn(
                    f'waiting for {self.odom_frame}->{self.base_frame}: {e}')
                self.warned = True
            return

        # T_map_odom = T_map_base * inverse(T_odom_base)
        t = odom_to_base.transform.translation
        r = odom_to_base.transform.rotation
        q_ob = (r.x, r.y, r.z, r.w)
        q_bo = q_inv(q_ob)
        # inverse translation: -R^-1 * t
        it = q_rotate(q_bo, (-t.x, -t.y, -t.z))

        q_mb = (pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w)
        q_mo = q_mult(q_mb, q_bo)
        off = q_rotate(q_mb, it)

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = pose.position.x + off[0]
        out.transform.translation.y = pose.position.y + off[1]
        out.transform.translation.z = pose.position.z + off[2]
        (out.transform.rotation.x, out.transform.rotation.y,
         out.transform.rotation.z, out.transform.rotation.w) = q_mo
        self.tf_broadcaster.sendTransform(out)


def main(args=None):
    rclpy.init(args=args)
    node = MirLocalizationTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
