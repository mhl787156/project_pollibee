#!/bin/python3

import sys
import argparse
import time
import threading
from typing import List
import rclpy
from as2_python_api.drone_interface import DroneInterface

parser=argparse.ArgumentParser(description="Starts gates mission for crazyswarm in either simulation or real environment")
parser.add_argument('-s', '--simulated', action='store_true', default=False)

speed = 1.0
ingore_yaw = True
height = 2.2
desp_gates = 0.5
v_dist = 3.0

initial_point_rel_gate_0 = [-v_dist/2, 0.0, 2.0]

initial_point_rel_gate_1 = [v_dist/2, 0.0, 2.0]

if desp_gates != 0.0:
    path_0 = [{"gate_0":[-desp_gates, 0.0, 2.0]}, {"gate_0":[desp_gates, 0.0 ,2.0]}, {"gate_1": initial_point_rel_gate_1}]
    path_1 = [{"gate_1":[desp_gates, 0.0, 2.0]}, {"gate_1":[-desp_gates, 0.0 ,2.0]}, {"gate_0": initial_point_rel_gate_0}]

else:
    path_0 = [{"gate_0":[0.0, 0.0 , 2.0]}, {"gate_1": initial_point_rel_gate_1}]
    path_1 = [{"gate_1":[0.0, 0.0 , 2.0]}, {"gate_0": initial_point_rel_gate_0}]

def confirm(uav: DroneInterface, msg: str = 'Continue') -> bool:
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    elif confirmation == "n":
        return False
    else:
        uav.shutdown()

def follow_path_with_go_to(drone_interface: DroneInterface):
    for frame_dict in path_0:
        for frame, point in frame_dict.items():
            drone_interface.go_to.go_to_point_path_facing(
                point=point, speed=speed, frame_id=frame)
    for frame_dict in path_1:
        for frame, point in frame_dict.items():
            drone_interface.go_to.go_to_point_path_facing(
                point=point, speed=speed, frame_id=frame)

def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    print("Start mission")

    ##### ARM OFFBOARD #####
    drone_interface.offboard()
    drone_interface.arm()

    ##### TAKE OFF #####
    if confirm(drone_interface, "Takeoff"):
        print("Take Off")
        drone_interface.takeoff(takeoff_height, speed=1.0)
        print("Take Off done")

    print("Initial Go To")
    if confirm(drone_interface, "Go To"):
        drone_interface.go_to.go_to_point_path_facing(
            point=initial_point_rel_gate_0, speed=speed, frame_id="gate_0")
    print("Follow Path")
    if confirm(drone_interface, "Follow Path"):
        follow_path_with_go_to(drone_interface)
    while confirm(drone_interface, "Replay"):
        follow_path_with_go_to(drone_interface)

    print("Land")
    if confirm(drone_interface, "Land"):
    ##### LAND #####
        print("Landing")
        drone_interface.land(speed=0.5)
        print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description="Starts gates mission for crazyswarm in either simulation or real environment")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)

    args = parser.parse_args()

    if args.simulated:
        print("Mission running in simulation mode")
    else:
        print("Mission running in real mode")

    rclpy.init()

    uav = DroneInterface(drone_id="cf0", verbose=False,
                         use_sim_time=args.simulated)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
