# Copyright 2020 Erwin Lejeune
# Under MIT Licence
# <erwin.lejeune15@gmail.com>

import rclpy
from rclpy.node import Node
import json
import pathlib
from pendulum_msgs_v2.msg import ControllerStats 
from pendulum_msgs_v2.msg import PendulumStats

class WriteJson(Node):

    def __init__(self):
        super().__init__('node')

        self.last_controller_msg = ''
        self.last_driver_msg = ''

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
 
        self.data_number_controller = 0
        self.data_number_driver = 0

        self.sub_controller_stats # prevent unused variable
        self.sub_pendulum_stats # prevent unused variable warning

    def controller_statistics_callback(self, msg):
        if str(msg) != self.last_controller_msg:
            data = self.convert_msg_to_dict(msg, self.data_number_controller)
            self.add_data_to_json(data, self.controller_stats_path)
            self.data_number_controller += 1
            self.last_controller_msg = str(msg)
    
    def driver_statistics_callback(self, msg):
        if str(msg) != self.last_driver_msg:
            data = self.convert_msg_to_dict(msg, self.data_number_driver)
            self.add_data_to_json(data, self.driver_stats_path)
            self.data_number_driver += 1
            self.last_driver_msg = str(msg)

    def create_json_files(self):
        empty_data = {}
        
        with open(self.driver_stats_path, 'w') as dfile:
            json.dump(empty_data, dfile)
        with open(self.controller_stats_path, 'w') as cfile:
            json.dump(empty_data, cfile)
    
    def convert_msg_to_dict(self, msg, rank):
        lst_msg = str(msg).split("pendulum_msgs_v2.msg.")
        msg_dict = {}
        msg_dict[rank] = {}
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
                    msg_dict[rank][fields_list[x][y]] = {}
                    for elem in key_value_list:
                        for l in elem:
                            if "stats" in l or l == "":
                                continue
                            else:
                                msg_dict[rank][fields_list[x][y]][elem[0]] = float(elem[1])

        return(msg_dict)

    def add_data_to_json(self, data, path):
        with open(path, "r+") as file:
            try:
                json_data = json.load(file)
                json_data.update(data)
                file.seek(0)
                json.dump(json_data, file, indent=4)
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
