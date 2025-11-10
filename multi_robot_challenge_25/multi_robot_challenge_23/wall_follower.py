#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import SetBool



def safe_min(seq, default=3.5):
    seq = [x for x in seq if math.isfinite(x)]
    return min(seq) if seq else default

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower_stable')
        # gjør servicenavn konfigurerbart og relativt til namespace
        self.declare_parameter('service_name', 'wall_follower_enable')
        svc_name = self.get_parameter('service_name').value
        # relativt navn (blir /<namespace>/wall_follower_enable når node kjøres i en namespace)
        self.wf_srv = self.create_service(SetBool, svc_name, self._wf_srv_cb)

        # relative topic names so they are namespaced when node is launched in /tb3_0 etc.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.clbk_laser, 10)

        self.d_des = 0.50 
        self.front_stop = 0.55
        self.lost_wall = 1.8

        self.k_d = 1.4
        self.k_a = 0.8
        self.v_max = 0.6
        self.curv_k = 2.0

        self.spin_dir = 0 
        self.spin_since = None
        self.spin_timeout = 2.5 

        self.state = 'FOLLOW' 
        self.find_dir = -1
        self.get_logger().info("WallFollower (stabil) kjører...")

        self.enabled = False

    def clbk_laser(self, msg: LaserScan):
     
        front = safe_min(list(msg.ranges[0:20]) + list(msg.ranges[-20:]), default=3.5)
        right = safe_min(msg.ranges[260:280], default=3.5)
        right_front = safe_min(msg.ranges[300:320], default=3.5)

        if front < self.front_stop:
            self.state = 'AVOID'
        elif right > self.lost_wall and right_front > self.lost_wall:
            self.state = 'FIND'
        else:
            self.state = 'FOLLOW'

        twist = Twist()

        if self.state == 'AVOID':
            twist.linear.x = 0.0
            twist.angular.z = +0.7
        elif self.state == 'FIND':
            twist.linear.x = 0.05
            twist.angular.z = 0.5 * self.find_dir
        else:
            theta = math.radians(45.0)
            alpha = math.atan2(right_front*math.cos(theta) - right,
                               right_front*math.sin(theta))
            dist_to_wall = right * math.cos(alpha)

            dist_err = self.d_des - dist_to_wall
            ang_err = -alpha

            omega = self.k_d * dist_err + self.k_a * ang_err

            v = self.v_max / (1.0 + self.curv_k*abs(omega))
            v = max(0.06, min(self.v_max, v))

            twist.linear.x = v
            twist.angular.z = omega

        self._anti_spin_watch(twist)

        self.cmd_pub.publish(twist)

    def _anti_spin_watch(self, twist: Twist):
        current_dir = 0
        if twist.angular.z > 0.15:
            current_dir = +1
        elif twist.angular.z < -0.15:
            current_dir = -1

        now = time.time()
        if current_dir == 0:
            self.spin_dir = 0
            self.spin_since = None
            return

        if self.spin_dir != current_dir:
            self.spin_dir = current_dir
            self.spin_since = now
            return

        if self.spin_since is None:
            self.spin_since = now
            return

        if now - self.spin_since > self.spin_timeout:
            twist.linear.x = 0.0
            twist.angular.z = -0.6 * self.spin_dir
            self.state = 'FIND'
            self.find_dir = -self.spin_dir

            self.spin_dir = 0
            self.spin_since = None
    
    def _wf_srv_cb(self, req, resp):
        self.enabled = bool(req.data)
        resp.success = True
        resp.message = 'wall_follower ' + ('enabled' if self.enabled else 'disabled')
        return resp

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
