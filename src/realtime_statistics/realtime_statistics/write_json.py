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
import pathlib
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
        
        
        default_path = str(pathlib.Path(__file__).parent.absolute())
        default_path = default_path.replace('/build/realtime_statistics/realtime_statistics', '')

        self.driver_stats_path = default_path + "/ros2_realtime_statistics/data/driver_stats.json"
        self.controller_stats_path = default_path + "/ros2_realtime_statistics/data//controller_stats.json"
        self.create_json_files()
 
        self.data_number = 0

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
        lst_msg = str(msg).split("pendulum_msgs_v2.msg.")
        msg_dict = {}
        msg_dict[self.data_number] = {}
        fields_list = []
        key_value_list = []
        lst_msg.pop(0)
        first_round = True

        ## Dump non relevant data
        for elem in lst_msg:
            if first_round:
                fields_list.append([lst_msg[0]])
                if ("ControllerStats(") in fields_list[0][0]:
                    fields_list[0][0] = fields_list[0][0].replace("ControllerStats(", "")
                elif ("PendulumStats(") in fields_list[0][0]:
                    fields_list[0][0] = fields_list[0][0].replace("PendulumStats(", "")
                first_round = False

            if ("TimerStats") in elem:
                elem = elem.replace("TimerStats", "")
            elif ("ControllerStats") in elem:
                elem = elem.replace("ControllerStats", "")
            elif ("PendulumStats") in elem:
                elem = elem.replace("PendulumStats", "")
            elif ("TopicStats") in elem:
                elem = elem.replace("TopicStats", "")
            elif("Rusage") in elem:
                elem = elem.replace("Rusage", "")
            
            if "(" in elem:
                elem = elem.replace("(", "")
            if ")" in elem:
                elem = elem.replace(")", "")
            if "" == elem:
                elem.remove(elem)
            if ' ' in elem:
                elem = elem.replace(" ", "")

            if (",") in elem:
                fields_list.append(elem.split(","))

        for elem in fields_list:
            for x in elem:
                if "=" in x:
                    key_value_list.append(x.split("="))

        for x in range(len(fields_list)):
            for y in range(len(fields_list[x])):
                if "stats" in fields_list[x][y]:
                    fields_list[x][y] = fields_list[x][y].replace(' ', '')
                    fields_list[x][y] = fields_list[x][y].replace('=', '')
                    msg_dict[self.data_number][fields_list[x][y]] = {}
                    for elem in key_value_list:
                        for l in elem:
                            if "stats" in l or l == "":
                                continue
                            else:
                                msg_dict[self.data_number][fields_list[x][y]][elem[0]] = float(elem[1])

        return(msg_dict)

    def add_data_to_json(self, data, path):
        with open(path, "r+") as file:
            try:
                json_data = json.load(file)
                json_data.update(data)
                file.seek(0)
                json.dump(json_data, file, indent=4)
                self.data_number += 1
            except json.decoder.JSONDecodeError:
                pass

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
