# Copyright 2020 Erwin Lejeune ; Sampreet Sarkar
# Under MIT Licence

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import json
import argparse
import random
import pathlib

def get_args():
    parser = argparse.ArgumentParser(
        description="Real-Time Statistics Plotter."
    )
    parser.add_argument("--mode",
                        type=str,
                        required=True,
                        help="Register 'controller' ; 'driver' ; 'both' stats",
                        default='controller')
    parser.add_argument("--plot",
                        type=str,
                        required=True,
                        help="Plot 'rt' or 'nrt' ?",
                        default="nrt")
    return parser.parse_args()

class PlotJson(Node):

    def __init__(self, mode, rt):
        super().__init__('node')

        self.mode = mode
        self.rt = rt

        default_path = str(pathlib.Path(__file__).parent.absolute())
        default_path = default_path.replace('/build/realtime_statistics/realtime_statistics', '')

        self.default_data_path = default_path + "/ros2_realtime_statistics/data/"

        if rt == 0:
            self.driver_stats_path = default_path + "/ros2_realtime_statistics/data/driver_nrt_stats.json"
            self.controller_stats_path = default_path + "/ros2_realtime_statistics/data/controller_nrt_stats.json"
        else:
            self.driver_stats_path = default_path + "/ros2_realtime_statistics/data/driver_rt_stats.json"
            self.controller_stats_path = default_path + "/ros2_realtime_statistics/data/controller_rt_stats.json"

    def read_json_file(self, path):
        with open(path, "r") as file:
            data = json.load(file)
        return data

    def plot_from_json(self):
        single = False
        if self.mode == 'controller':
            label = "Controller"
            json_data = self.read_json_file(self.controller_stats_path)
            dict_json_sorted, nlist = self.sorted_data_from_json(json_data)
            single = True
        elif self.mode == 'driver':
            label = "Driver"
            json_data = self.read_json_file(self.driver_stats_path)
            dict_json_sorted, nlist = self.sorted_data_from_json(json_data)
            single = True
        elif self.mode == 'both':
            label = "Driver and Controller"
            c_json_data = self.read_json_file(self.controller_stats_path)
            d_json_data = self.read_json_file(self.driver_stats_path)
            c_dict_json_sorted, c_nlist = self.sorted_data_from_json(c_json_data)
            d_dict_json_sorted, d_nlist = self.sorted_data_from_json(d_json_data)
        else:
            print("Error : mode not supported")
            self.destroy_node()

        if single:
            self.plot_from_sorted_data(nlist, label, dict_json_sorted)
        else:
            self.plot_from_sorted_data(c_nlist, "Controller", c_dict_json_sorted)
            self.plot_from_sorted_data(d_nlist, "Driver", d_dict_json_sorted)
        plt.show()
    
    def sorted_data_from_json(self, json_data):
        dict_json_sorted = dict()
        n_pts = range(0, int(list(json_data.keys())[-1]) + 1)
        for number_key in json_data:
            for category in json_data[number_key]:
                dict_json_sorted[category] = dict()
                for data in json_data[number_key][category]:
                    dict_json_sorted[category][data] = []

        for number_key in json_data:
            for category in json_data[number_key]:
                for data in json_data[number_key][category]:
                    dict_json_sorted[category][data].append(json_data[number_key][category][data])

        return dict_json_sorted, n_pts
    
    def plot_from_sorted_data(self, n_pts_list, label, dict_json_sorted):
        rt_label = ""
        if self.rt == 0:
            rt_label = "(NRT Linux) "
        elif self.rt == 1: 
            rt_label = "(Xenomai) "
        else:
            print("Error, RT mode should be 0 or 1")
            exit()

        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle("Real Time " + rt_label + label + " Statistics - Inverted Pendulum")
        rdn_category_key = random.choice(list(dict_json_sorted.keys()))
        self.axs[0, 0].plot(n_pts_list, dict_json_sorted[rdn_category_key]["jitter_mean_usec"])
        self.axs[0, 0].set_title("Mean Jitters (µs)")
        self.axs[0, 1].plot(n_pts_list, dict_json_sorted[rdn_category_key]["jitter_std_usec"])
        self.axs[0, 1].set_title("STD Jitters (µs)")
        self.axs[1, 1].plot(n_pts_list, dict_json_sorted[rdn_category_key]["involuntary_context_switches"])
        self.axs[1, 1].set_title("Involuntary Context Switches")
        self.axs[1, 0].plot(n_pts_list, dict_json_sorted[rdn_category_key]["jitter_max_usec"])
        self.axs[1, 0].set_title("Max Jitters (µs)")
        if (self.rt == 0):
            plt.savefig(self.default_data_path + "nrt_" + label + '.png')
        else:
            plt.savefig(self.default_data_path + "rt_" + label + '.png')

def main(args=None):
    argprs = get_args()
    rclpy.init(args=args)
    if argprs.plot == 'rt':
        rt = 1
    elif argprs.plot == 'nrt':
        rt = 0
    plt_json = PlotJson(argprs.mode, rt)
    plt_json.plot_from_json()
    rclpy.spin_once(plt_json)

    # Destroy the node explicitly
    # (cmdional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plt_json.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
