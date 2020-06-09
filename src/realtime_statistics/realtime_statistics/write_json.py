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
import json
import yaml
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
        
        self.driver_stats_path = "../../resource/driver_stats.json"
        self.controller_stats_path = "../../resource/controller_stats.json"
        
        self.create_json_files()

        self.sub_controller_stats # prevent unused variable
        self.sub_pendulum_stats # prevent unused variable warning

    def controller_statistics_callback(self, msg):
        data = self.convert_msg_to_dict(msg)
        self.add_data_to_json(data, self.controller_stats_path)
    
    def driver_statistics_callback(self, msg):
        data = self.convert_msg_to_dict(msg)
        self.add_data_to_json(data, self.driver_stats_path)
    
    def create_json_files(self):
        empty_data = {}
        with open(self.driver_stats_path, 'w') as dfile:
            json.dump(empty_data, dfile)
        with open(self.controller_stats_path, 'w') as cfile:
            json.dump(empty_data, cfile)
    
    def convert_msg_to_dict(self, msg):
        yaml_data = yaml.load(str(msg))
        return yaml_data

    def add_data_to_json(self, data, path):
        with open(path, "r+") as file:
            json_data = json.load(file)
            json_data.update(data)
            file.seek(0)
            json.dump(data, file, indent=4)


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
