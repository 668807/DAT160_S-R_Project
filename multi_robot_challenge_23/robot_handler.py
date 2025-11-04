#robot_handler.py
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

class RobotHandlerClass(Node):
    def __init__(self):
        super().__init__('RobotHandlerNode')

        # sjekk de 2 linjene under her
        self.robot_name = self.get_namespace().strip('/')
        self.get_logger().info(f'Robot Handler Node started for robot: {self.robot_name}')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.clbk_lidar,
            10
        )

        self.namespace_test_pub = self.create_publisher(
            Float64,
            'namespace_test',
            10
        )

        self.lidar_value = 100.0
        timer_period = 1.0  # seconds
        # Create timer function that gets executed once per second
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Callback function for lidar subscriber
    def clbk_lidar(self, msg):
        # Safte the lidar value at 180 degrees in a class wide variable
        self.lidar_value = msg.ranges[180]


    # timer function that is executed 1 time per second
    def timer_callback(self):
        # Create a message of type Float64
        pub_msg = Float64()
        # Fill the data object of the message with the lidar value saved in the lidar subscriber callback function
        pub_msg.data = self.lidar_value


        self.namespace_test_pub.publish(pub_msg)




def main(args=None):
    rclpy.init(args=args)

    robot_handler = RobotHandlerClass()

    rclpy.spin(robot_handler)

    robot_handler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


