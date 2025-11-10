#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose # Import Pose for the subscriber

from std_srvs.srv import SetBool
from bug2_interfaces.srv import GoToPoint as GoToPointSrv


class Bug2Controller(Node):
    """
    Bug2 controller:
      - Topics: /odom, /scan, /mission/fire_target_pose
      - Services: /wall_follower_enable, /go_to_point_switch
    """

    def __init__(self):
        super().__init__('bug2_controller')

        # Fjernet hardkodede målparametre: goal_x, goal_y

        # Parametere for service-navn og terskler (beholdt)
        self.declare_parameter('wall_follower_service_name', 'wall_follower_enable')
        self.declare_parameter('go_to_point_service_name', 'go_to_point_switch')
        self.declare_parameter('obstacle_distance_threshold', 0.5)
        self.declare_parameter('line_hit_tolerance', 0.10)
        self.declare_parameter('goal_tolerance', 0.20)

        self.wall_srv_name = self.get_parameter('wall_follower_service_name').value
        self.gtp_srv_name = self.get_parameter('go_to_point_service_name').value
        self.obs_thr = float(self.get_parameter('obstacle_distance_threshold').value)
        self.line_tol = float(self.get_parameter('line_hit_tolerance').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)

        # Tilstandsvariabler
        self.state = 'WAIT_FOR_GOAL'  # <-- NY STARTTILSTAND
        self.start_point: Optional[Point] = None
        self.current_pose: Optional[Point] = None
        self.goal: Optional[Point] = None # <-- Målet er nå OPTIONAL
        self.left_line_goal_dist: Optional[float] = None
        self.obstacle_in_front: bool = False
        self.last_switch_time = self.get_clock().now()

        # Subscriptions
        # Lytter til global målposisjon (Big Fire)
        self.sub_fire_goal = self.create_subscription(
            Pose, 
            'mission/fire_target_pose', 
            self.cb_fire_target_pose, 
            10
        )
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.sub_scan = self.create_subscription(LaserScan, 'scan', self.cb_scan, 10)

        # Clients
        self.wall_client = self.create_client(SetBool, self.wall_srv_name)
        self.gtp_client = self.create_client(GoToPointSrv, self.gtp_srv_name)

        self.get_logger().info('Bug2Controller started, waiting for goal...')

        # timer for control loop
        self.control_timer = self.create_timer(0.2, self.control_step)

    # ------------------ NYTT: Mål-mottaker Callback ------------------
    def cb_fire_target_pose(self, msg: Pose):
        """Mottar målposisjonen fra Robot Handler."""
        if self.state == 'WAIT_FOR_GOAL':
            # Bruk posisjonsdelen av Pose-meldingen som målet
            self.goal = msg.position
            self.get_logger().info(
                f'Goal received! Starting navigation to ({self.goal.x:.2f}, {self.goal.y:.2f})'
            )
            # Begynn navigasjon
            self.enable_go_to_point(True, self.goal)
            self.state = 'GO_TO_POINT'
            self.last_switch_time = self.get_clock().now()
        # Ignorer nye mål hvis vi allerede navigerer
        # Kan legges til logikk for å oppdatere mål hvis ønskelig.

    # ------------------ Endret logikk i cb_odom ------------------
    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_pose = Point(x=p.x, y=p.y, z=0.0)
        
        # Setter startpunkt KUN når vi er klare til å starte navigasjonen
        if self.start_point is None and self.state != 'WAIT_FOR_GOAL':
            self.start_point = Point(x=p.x, y=p.y, z=0.0)
            self.get_logger().info(
                f'Start point fixed at ({self.start_point.x:.2f}, {self.start_point.y:.2f})'
            )

    # ------------------ Endret logikk i control_step ------------------
    def control_step(self):
        # Må ha mål, aktuell posisjon og startposisjon for å kjøre
        if self.state == 'DONE' or self.goal is None or self.current_pose is None or self.start_point is None:
            if self.state == 'GO_TO_POINT' or self.state == 'WALL_FOLLOW':
                # Hvis vi er i en aktiv tilstand, men mistet pos/startpunkt (skal ikke skje i praksis)
                self.get_logger().warn("Active state but missing goal or position data.")
            return

        # Sjekk om mål nådd (bruker dist_to_goal som krever self.goal)
        if self.dist_to_goal(self.current_pose) < self.goal_tol:
            self.get_logger().info('Goal reached. Stopping all controllers.')
            self.enable_go_to_point(False, self.goal)
            self.enable_wall_follower(False)
            self.state = 'DONE'
            return

        d_line = self.point_to_line_distance(self.current_pose, self.start_point, self.goal)

        if self.state == 'GO_TO_POINT':
            if self.obstacle_in_front and self.cooldown_elapsed(0.8):
                self.left_line_goal_dist = self.dist_to_goal(self.current_pose)
                self.get_logger().info(
                    f'Obstacle detected — switching to WALL_FOLLOW (dist={self.left_line_goal_dist:.2f})'
                )
                self.enable_go_to_point(False, self.goal)
                self.enable_wall_follower(True)
                self.state = 'WALL_FOLLOW'
                self.last_switch_time = self.get_clock().now()

        elif self.state == 'WALL_FOLLOW':
            closer = (
                self.left_line_goal_dist is not None and
                self.dist_to_goal(self.current_pose) < self.left_line_goal_dist
            )
            on_line = d_line <= self.line_tol
            if on_line and closer and self.cooldown_elapsed(1.0):
                self.get_logger().info(
                    f'Back on M-line closer to goal (d_line={d_line:.2f}). Switching to GO_TO_POINT.'
                )
                self.enable_wall_follower(False)
                self.enable_go_to_point(True, self.goal)
                self.state = 'GO_TO_POINT'
                self.last_switch_time = self.get_clock().now()
        
        # --- Uendrede hjelpemetoder ---
    def cb_scan(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            self.obstacle_in_front = False
            return
        span = max(1, int(15 / 360.0 * n))
        front_ranges = list(msg.ranges[0:span]) + list(msg.ranges[-span:])
        valid = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        min_front = min(valid, default=float('inf'))
        self.obstacle_in_front = (min_front < self.obs_thr)


    def dist(self, a: Point, b: Point) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def dist_to_goal(self, p: Point) -> float:
        # Sikkerhetsjekk: Returner stor verdi hvis mål ikke er satt
        if self.goal is None:
             return float('inf')
        return self.dist(p, self.goal)

    def point_to_line_distance(self, p: Point, a: Point, b: Point) -> float:
        """Perpendicular distance from point p to line a→b."""
        ax, ay, bx, by, px, py = a.x, a.y, b.x, b.y, p.x, p.y
        dx, dy = bx - ax, by - ay
        denom = math.hypot(dx, dy)
        if denom < 1e-6:
            return 0.0
        return abs(dy * px - dx * py + bx * ay - by * ax) / denom

    def cooldown_elapsed(self, sec: float) -> bool:
        return (self.get_clock().now() - self.last_switch_time) >= Duration(seconds=sec)

    def enable_wall_follower(self, enable: bool):
        if not self.wall_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Wall follower service not ready.')
            return
        req = SetBool.Request()
        req.data = enable
        try:
            future = self.wall_client.call_async(req)
            future.add_done_callback(lambda f: self._log_srv_result('wall_follower', f, enable))
        except Exception as e:
            self.get_logger().error(f'Failed to call wall_follower service: {e}')

    def enable_go_to_point(self, enable: bool, target: Point):
        if not self.gtp_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Go-to-point service not ready.')
            return
        req = GoToPointSrv.Request()
        req.move_switch = enable
        req.target_position = target
        try:
            future = self.gtp_client.call_async(req)
            future.add_done_callback(lambda f: self._log_srv_result('go_to_point', f, enable))
        except Exception as e:
            self.get_logger().error(f'Failed to call go_to_point service: {e}')

    def _log_srv_result(self, name: str, future, enable: bool):
        try:
            res = future.result()
            ok = getattr(res, 'success', True)
            self.get_logger().info(f'[{name}] -> {"ENABLED" if enable else "DISABLED"} (success={ok})')
        except Exception as e:
            self.get_logger().error(f'[{name}] service call failed: {e}')
        
# ------------------ Main-funksjon (uendret) ------------------

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Sikkerhetsstopp ved avslutning (hvis mål er satt)
        if node.goal:
            node.enable_go_to_point(False, node.goal)
            node.enable_wall_follower(False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
