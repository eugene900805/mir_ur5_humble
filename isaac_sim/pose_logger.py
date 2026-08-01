"""Log Isaac ground truth (/odom) against AMCL (/amcl_pose) so localisation
error can be separated from control error in the narrow corridor."""
import math, csv, sys, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.z*q.z+q.y*q.y))

class L(Node):
    def __init__(self, path):
        super().__init__('pose_logger')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'amcl_x', 'amcl_y', 'amcl_yaw'])
        self.gt = None
        self.create_subscription(Odometry, '/odom', self.on_gt, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)

    def on_gt(self, m):
        p = m.pose.pose
        self.gt = (m.header.stamp.sec + m.header.stamp.nanosec*1e-9,
                   p.position.x, p.position.y, yaw_of(p.orientation))

    def on_amcl(self, m):
        if self.gt is None:
            return
        p = m.pose.pose
        self.w.writerow([f'{self.gt[0]:.3f}', f'{self.gt[1]:.4f}', f'{self.gt[2]:.4f}',
                         f'{self.gt[3]:.4f}', f'{p.position.x:.4f}', f'{p.position.y:.4f}',
                         f'{yaw_of(p.orientation):.4f}'])
        self.f.flush()

rclpy.init()
n = L(sys.argv[1])
rclpy.spin(n)
