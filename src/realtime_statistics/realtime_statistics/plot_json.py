# Copyright 2020 Erwin Lejeune ; Sampreet Sarkar
# Under MIT Licence

import rclpy
from rclpy.node import Node
from pendulum_msgs_v2.msg import ControllerStats 
from pendulum_msgs_v2.msg import PendulumStats

class WriteJson(Node):

    def __init__(self):
        super().__init__('node')
        self.sub_controller_stats = self.create_subscription(
            ControllerStats,
            '/controller_statistics',
            self.controller_statistics_callback,
            10)
        self.sub_pendulum_stats = self.create_subscription(
            PendulumStats,
            '/driver_statistics',
            self.driver_statistics_callback,
            10)
        
        self.create_json_file()

        self.sub_controller_stats # prevent unused variable
        self.sub_pendulum_stats # prevent unused variable warning

    def controller_statistics_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def driver_statistics_callback(self, msg):
        x = 10
    
    def create_json_file(self):
        # lol
        x = 10
    
    def add_data_to_json(self):
        # lol
        x = 10


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = WriteJson()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
