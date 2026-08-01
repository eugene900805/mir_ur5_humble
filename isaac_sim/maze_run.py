"""Send a NavigateToPose goal that forces the robot through the maze's 0.93 m
corridor (x 6.02..6.96, y 2.08..3.39) and log the Isaac ground-truth pose."""
import math, sys, time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry

GX, GY, GYAW = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])) \
    if len(sys.argv) > 3 else (5.5, 5.0, math.pi / 2)
TIMEOUT = float(sys.argv[4]) if len(sys.argv) > 4 else 240.0

class Run(Node):
    def __init__(self):
        super().__init__('maze_run')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.track = []
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.result = None
        self.done = False

    def odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.z*q.z+q.y*q.y))
        t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if not self.track or t - self.track[-1][0] > 0.25:
            self.track.append((t, p.x, p.y, p.z, yaw))

    def go(self):
        self.ac.wait_for_server()
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x = GX
        g.pose.pose.position.y = GY
        g.pose.pose.orientation.z = math.sin(GYAW/2)
        g.pose.pose.orientation.w = math.cos(GYAW/2)
        f = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f)
        h = f.result()
        if not h.accepted:
            print('GOAL REJECTED'); return
        print(f'goal accepted -> ({GX}, {GY})', flush=True)
        rf = h.get_result_async()
        rf.add_done_callback(self.on_result)
        t0 = time.time()
        while not self.done and time.time()-t0 < TIMEOUT:
            rclpy.spin_once(self, timeout_sec=0.2)
        self.report()

    def on_result(self, fut):
        self.result = fut.result().status
        self.done = True

    def report(self):
        STATUS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        print('result:', STATUS.get(self.result, f'TIMEOUT/{self.result}'))
        tr = self.track
        if not tr:
            print('no odom'); return
        print(f'samples={len(tr)} sim time={tr[-1][0]-tr[0][0]:.1f}s')
        print(f'start ({tr[0][1]:.2f}, {tr[0][2]:.2f})  end ({tr[-1][1]:.2f}, {tr[-1][2]:.2f})')
        # corridor transit
        inside = [s for s in tr if 6.02 < s[1] < 6.96 and 2.08 < s[2] < 3.39]
        north = [s for s in tr if s[2] > 3.5]
        print(f'samples inside the 0.93 m corridor: {len(inside)}')
        if inside:
            xs = [s[1] for s in inside]
            # Worst gap between the BODY (not base_footprint's origin) and a
            # wall: the walls are at x=6.023 / 6.956 and the MiR100 is 0.58 m
            # wide, so subtract the 0.29 m half-width. Centred gives 17.6 cm;
            # every measured stall so far happened below ~6 cm.
            body = min(min(xs) - 6.023, 6.956 - max(xs)) - 0.29
            print(f'  x range in corridor: {min(xs):.3f} .. {max(xs):.3f} '
                  f'(centre 6.490, body clearance {body*100:.1f} cm of 17.6 max)')
        print(f'reached the north side (y>3.5): {"YES" if north else "NO"}')
        zmax = max(abs(s[3]) for s in tr)
        print(f'max |z| of base (ejection check): {zmax:.3f} m')
        dmax = max(math.hypot(s[1], s[2]) for s in tr)
        print(f'max distance from origin: {dmax:.2f} m (map is +-10 m)')

rclpy.init()
r = Run(); r.go()
