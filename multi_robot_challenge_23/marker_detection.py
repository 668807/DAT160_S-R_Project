import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64
from geometry_msgs.msg import Pose, Point
from scoring_interfaces.srv import SetMarkerPosition

class MarkerDetection(Node):
    def __init__(self):
        super().__init__('MarkerDetectionNode')

        self.declare_parameter('namespace', 'tb3_0')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.get_logger().info(f'Marker Detection Node started for namespace: {self.namespace}')

        self.sub_pose = self.create_subscription(
            Pose,
            '/' + self.namespace + '/marker_map_pose',
            self.clbk_marker_map_pose,
            10
        )
        # The node marker_pose.py publishes the ID as an Int64
        self.sub_id = self.create_subscription(
            Int64,
            '/' + self.namespace + '/marker_id',
            self.clbk_marker_id,
            10
        )

        # Default values for variables
        self.prev_marker_id = 1000
        self.marker_id = 1000
        self.marker_position = Point()
        self.reported_markers = set()

        self.score_client = self.create_client(
            SetMarkerPosition, 
            'set_marker_position'
            )
        self.get_logger().info('Waiting for scoring service...')
        while not self.score_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Scoring service connected.')

        timer_period = 1.0  # seconds
        # Create timer function that gets executed once per second
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def clbk_marker_map_pose(self, msg):
        self.marker_position = msg.position

    def clbk_marker_id(self, msg):
        self.marker_id = msg.data

    def call_scoring_service(self):
        req = SetMarkerPosition.Request()
        req.marker_id = self.marker_id
        req.marker_position = self.marker_position

        future = self.score_client.call_async(req)

        future.add_done_callback(self.scoring_response_callback)

    def scoring_response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Scoring service call failed: {e}')
            return
        
        if response.accepted:
            self.get_logger().info(f'Marker ID {self.marker_id} position accepted for scoring.')
            self.reported_markers.add(self.marker_id)
        else:
            self.get_logger().info(f'Marker ID {self.marker_id} position rejected for scoring.')

    def timer_callback(self):
        if self.marker_id != self.prev_marker_id and self.marker_id != 1000:
            if self.marker_id not in self.reported_markers:
                pos = self.marker_position
                self.get_logger().info(f'New marker detected. ID: {self.marker_id}')
                self.call_scoring_service()
            else:
                self.get_logger().info(f'Marker ID {self.marker_id} has already been reported.')

            pos = self.marker_position
            self.get_logger().info(f"Marker ID: {self.marker_id} Position - x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")  

        self.prev_marker_id = self.marker_id   

       
def main(args=None):
    rclpy.init(args=args)

    marker_detection = MarkerDetection()

    rclpy.spin(marker_detection)

    marker_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
