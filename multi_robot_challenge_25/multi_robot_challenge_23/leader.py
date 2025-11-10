import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

class LeaderClass(Node):
    def init(self):
        super().init('RobotLeaderNode')
        self.get_logger().info('Robot Leader Node started. Subscribing to handler data.')

        #SUBSCRIBES to the namespaced topic published by tb3_0's RobotHandler
        self.sub_tb3_0 = self.create_subscription(
            Float64,
            '/tb3_0/lidar_value_out',  # Updated topic name
            self.clbk_tb3_0_test,
            10
        )

        #SUBSCRIBES to the namespaced topic published by tb3_1's RobotHandler
        self.sub_tb3_1 = self.create_subscription(
            Float64,
            '/tb3_1/lidar_value_out',  # Updated topic name
            self.clbk_tb3_1_test,
            10
        )

        self.tb3_0_lidar_value = 1000.0
        self.tb3_1_lidar_value = 1000.0

        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def clbk_tb3_0_test(self, msg):
        self.tb3_0_lidar_value = msg.data

    def clbk_tb3_1_test(self, msg):
        self.tb3_1_lidar_value = msg.data

    def timer_callback(self):
        self.get_logger().info(f'TB3_0 Lidar Value: {self.tb3_0_lidar_value:.2f}')
        self.get_logger().info(f'TB3_1 Lidar Value: {self.tb3_1_lidar_value:.2f}')


def main(args=None):
    rclpy.init(args=args)
    robot_leader = LeaderClass()
    rclpy.spin(robot_leader)
    robot_leader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

