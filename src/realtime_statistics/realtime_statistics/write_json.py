# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

    writejson = WriteJson()

    rclpy.spin(writejson)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    writejson.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
