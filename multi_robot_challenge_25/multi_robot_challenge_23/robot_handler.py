import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64, Int64
from geometry_msgs.msg import Pose, Point

class RobotHandlerClass(Node):
    def __init__(self):
        super().__init__('RobotHandlerNode')

        # Egen namespace, f.eks. 'tb3_0'
        self.robot_name = self.get_namespace().strip('/')
        self.get_logger().info(f'Robot Handler Node started for robot: {self.robot_name}')

        # === Parametre ===
        # Hvilken robot skal få Bug2-målet når vi oppdager en ArUco?
        self.declare_parameter('target_robot_ns', 'tb3_1')
        self.target_robot_ns = str(self.get_parameter('target_robot_ns').value).strip('/')

        # Hvilke marker-IDer vil vi trigge på? (tom liste -> alle)
        self.declare_parameter('trigger_marker_ids', [4])
        self.trigger_marker_ids = set(self.get_parameter('trigger_marker_ids').value or [])

        # === Subscribers ===
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.clbk_lidar, 10)

        # ArUco-topics (publisert av din marker-deteksjon på denne roboten):
        #  - '/<ns>/marker_id'        : std_msgs/Int64
        #  - '/<ns>/marker_map_pose'  : geometry_msgs/Pose  (posisjon i kart/verdenskoordinater)
        self.sub_marker_id = self.create_subscription(Int64, 'marker_id', self.clbk_marker_id, 10)
        self.sub_marker_pose = self.create_subscription(Pose, 'marker_map_pose', self.clbk_marker_pose, 10)

        # === Publishers ===
        # Egen posisjon (beholdt)
        self.pose_pub  = self.create_publisher(Pose, 'robot_pose', 10)
        # Global mission (ikke brukt lenger)
        self.fire_pub  = self.create_publisher(Pose, '/mission/fire_target_pose', 10)
        # Navngitt, namespacet lidar-verdi (som før)
        self.lidar_pub = self.create_publisher(Float64, 'lidar_value_out', 10)

        # Viktig: mål-robotens *namespacede* mission-kanal:
        self.target_mission_pub = self.create_publisher(
            Pose,
            f'/{self.target_robot_ns}/mission/fire_target_pose',
            10
        )

        # Tilstand
        self.lidar_value = 100.0
        self.current_marker_id = None
        self.current_marker_pose = None
        self.reported_ids = set()   # husker hvilke IDer vi har trigget allerede

        # Timer for demo/logging
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(
            f"ArUco routing enabled: when '{self.robot_name}' sees a tag -> "
            f"send goal to '/{self.target_robot_ns}/mission/fire_target_pose'"
        )

    # ===== ArUco callbacks =====
    def clbk_marker_id(self, msg: Int64):
        self.current_marker_id = int(msg.data)
        self._maybe_publish_goal()

    def clbk_marker_pose(self, msg: Pose):
        self.current_marker_pose = msg
        self._maybe_publish_goal()

    def _maybe_publish_goal(self):
        # Trenger både ID og pose
        if self.current_marker_id is None or self.current_marker_pose is None:
            return

        marker_id = self.current_marker_id

        # Filtrer på ønskede IDer (hvis satt)
        if self.trigger_marker_ids and marker_id not in self.trigger_marker_ids:
            return

        # Unngå dobbeltrapportering
        if marker_id in self.reported_ids:
            return

        # Publiser målet til mål-robotens mission-topic
        goal_pose = Pose()
        goal_pose.position = Point(
            x=self.current_marker_pose.position.x,
            y=self.current_marker_pose.position.y,
            z=0.0
        )
        goal_pose.orientation.w = 1.0  # nøytral

        self.target_mission_pub.publish(goal_pose)
        self.reported_ids.add(marker_id)
        self.get_logger().info(
            f"ArUco ID {marker_id} detected by {self.robot_name} -> "
            f"sent goal to '/{self.target_robot_ns}/mission/fire_target_pose' "
            f"({goal_pose.position.x:.2f}, {goal_pose.position.y:.2f})"
        )

    # ===== Lidar demo (som før) =====
    def clbk_lidar(self, msg: LaserScan):
        self.lidar_value = msg.ranges[len(msg.ranges)//2] if msg.ranges else float('inf')

    def timer_callback(self):
        pub_msg = Float64()
        pub_msg.data = float(self.lidar_value)
        self.lidar_pub.publish(pub_msg)

def main(args=None):
    rclpy.init(args=args)
    robot_handler = RobotHandlerClass()
    rclpy.spin(robot_handler)
    robot_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
