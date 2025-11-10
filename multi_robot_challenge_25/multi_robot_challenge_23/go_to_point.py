# bug2_navigation/go_to_point.py
import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

from bug2_interfaces.srv import GoToPoint as GoToPointSrv


def normalize_angle(a: float) -> float:
    # map vinkel til [-pi, pi]
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a < 0:
        a += 2.0 * math.pi
    return a - math.pi


class GoToPoint(Node):
    def __init__(self):
        super().__init__('go_to_point')

        # --- Pub/Sub ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.clbk_odom, 10)

        self.declare_parameter('use_front_stop', False)
        self.use_front_stop = self.get_parameter('use_front_stop').get_parameter_value().bool_value
        self.front_stop_threshold = 0.35
        self.front_dist = float('inf')
        if self.use_front_stop:
            self.scan_sub = self.create_subscription(LaserScan, 'scan', self.clbk_scan, 5)

        # Parametre
        self.declare_parameter('lin_speed', 0.2)
        self.declare_parameter('ang_speed', 0.8)
        self.declare_parameter('kp_ang', 1.8)
        self.declare_parameter('kp_strafe', 1.2)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('dist_tolerance', 0.10)

        self.lin_speed = self.get_parameter('lin_speed').get_parameter_value().double_value
        self.ang_speed = self.get_parameter('ang_speed').get_parameter_value().double_value
        self.kp_ang = self.get_parameter('kp_ang').get_parameter_value().double_value
        self.kp_strafe = self.get_parameter('kp_strafe').get_parameter_value().double_value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').get_parameter_value().double_value
        self.dist_tolerance = self.get_parameter('dist_tolerance').get_parameter_value().double_value

        # Standardmål 
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', -10.0)
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value

        # Tilstand 
        self.position = None
        self.yaw = 0.0
        self.state = 0
        self.enabled = False
        self._prev_enabled = False 

        self.srv = self.create_service(GoToPointSrv, 'go_to_point_switch', self._srv_cb)
        self.get_logger().info('Service /go_to_point_switch (bug2_interfaces/GoToPoint) ready.')

        self.timer = self.create_timer(0.05, self.control_loop)

    # Service callback 
    def _srv_cb(self, req: GoToPointSrv.Request, resp: GoToPointSrv.Response):
        self.enabled = bool(req.move_switch)
        if self.enabled:
            self.goal_x = float(req.target_position.x)
            self.goal_y = float(req.target_position.y)
            self.state = 0 
            self.get_logger().info(
                f'GoToPoint ENABLED. New goal=({self.goal_x:.2f}, {self.goal_y:.2f})'
            )
        else:
            self.get_logger().info('GoToPoint DISABLED.')

        resp.success = True
        return resp

    # Callbacks
    def clbk_scan(self, msg: LaserScan):
        rng = np.array(msg.ranges, dtype=float)
        n = len(rng)
        front = rng[int(n * 0.4):int(n * 0.6)]
        front = [x for x in front if np.isfinite(x)]
        self.front_dist = min(front) if front else float('inf')

    def clbk_odom(self, msg: Odometry):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quaternion = (q.x, q.y, q.z, q.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def control_loop(self):
        if self.position is None:
            return

        if not self.enabled:
            if self._prev_enabled:
                self._publish_stop()
            self._prev_enabled = False
            return

        self._prev_enabled = True

        cmd = Twist()

        dx = self.goal_x - self.position.x
        dy = self.goal_y - self.position.y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = normalize_angle(target_yaw - self.yaw)

        if self.state == 0:
            if abs(yaw_err) <= self.yaw_tolerance:
                self.state = 1
        elif self.state == 1:
            if dist <= self.dist_tolerance:
                self.state = 2
            elif abs(yaw_err) > 2.0 * self.yaw_tolerance:
                self.state = 0
        elif self.state == 2:
            pass

        if self.use_front_stop and self.front_dist < self.front_stop_threshold:
            self.get_logger().warn('Obstacle ahead - stopping!')
            self.state = 0
            self._publish_stop()
            return

        # Kontroller
        if self.state == 0:  # roter mot mål
            ang = self.kp_ang * yaw_err
            cmd.angular.z = float(np.clip(ang, -self.ang_speed, self.ang_speed))
            cmd.linear.x = 0.0

        elif self.state == 1:  # kjør mot mål
            ang = self.kp_strafe * yaw_err
            cmd.angular.z = float(np.clip(ang, -self.ang_speed, self.ang_speed))
            cmd.linear.x = self.lin_speed

        elif self.state == 2:  # fremme
            self._publish_stop()
            return

        self.cmd_pub.publish(cmd)

    def _publish_stop(self):
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)

def main():
    rclpy.init()
    node = GoToPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
