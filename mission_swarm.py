#!/bin/python3

import sys
import argparse
import time
import threading
from typing import List
import math
import rclpy
from copy import deepcopy
from tf2_ros.buffer_interface import TransformRegistration
from tf2_geometry_msgs import PointStamped, TransformStamped
from as2_python_api.drone_interface import DroneInterface

# pos= [[1,0,1],[-1,1,1.5],[-1,-1,2.0]]
parser=argparse.ArgumentParser(description="Starts gates mission for crazyswarm in either simulation or real environment")
parser.add_argument('-s', '--simulated', action='store_true', default=False)

speed = 1.0
ingore_yaw = True
height = 2.2
desp_gates = 0.5
drones_ns = ['cf0', 'cf1']

v_dist = 3.0

initial_point_rel_gate_0 = [-v_dist/2, 0.0, 2.0]

initial_point_rel_gate_1 = [v_dist/2, 0.0, 2.0]

if desp_gates != 0.0:
    path_0 = [{"gate_0":[-desp_gates, 0.0, 2.0]}, {"gate_0":[desp_gates, 0.0 ,2.0]}, {"gate_1": initial_point_rel_gate_1}]
    path_1 = [{"gate_1":[desp_gates, 0.0, 2.0]}, {"gate_1":[-desp_gates, 0.0 ,2.0]}, {"gate_0": initial_point_rel_gate_0}]

else:

    path_0 = [{"gate_0":[0.0, 0.0 , 2.0]}, {"gate_1": initial_point_rel_gate_1}]
    path_1 = [{"gate_1":[0.0, 0.0 , 2.0]}, {"gate_0": initial_point_rel_gate_0}]


def shutdown_all(uavs):
    print("Exiting...")
    for uav in uavs:
        uav.shutdown()
    sys.exit(1)

# create decorator for creating a thread for each drone


def takeoff(uav: DroneInterface):
    uav.arm()
    uav.offboard()
    uav.takeoff(2.0, 0.7)
    time.sleep(1)


def land(drone_interface: DroneInterface):
    # position = drone_interface.position
    # land_position = [
    #     position[0],
    #     position[1],
    #     0.2
    # ]
    drone_interface.land(0.5)

def follow_path_with_go_to(drone_interface: DroneInterface):
    if drone_interface.drone_id == drones_ns[0]:
        for frame_dict in path_0:
            for frame, point in frame_dict.items():
                drone_interface.go_to.go_to_point_path_facing(
                    point=point, speed=speed, frame_id=frame)
            
        for frame_dict in path_1:
            for frame, point in frame_dict.items():
                drone_interface.go_to.go_to_point_path_facing(
                    point=point, speed=speed, frame_id=frame)
            
    elif drone_interface.drone_id == drones_ns[1]:
        for frame_dict in path_1:
            for frame, point in frame_dict.items():
                drone_interface.go_to.go_to_point_path_facing(
                    point=point, speed=speed, frame_id=frame)
            
        for frame_dict in path_0:
            for frame, point in frame_dict.items():
                drone_interface.go_to.go_to_point_path_facing(
                    point=point, speed=speed, frame_id=frame)

        
    # drone_interface.goto.go_to_point_path_facing(o
    #     pose_generator(drone_interface), speed=speed)


def initial_go_to(drone_interface: DroneInterface):
    if (drone_interface.drone_id == drones_ns[0]):
        point = initial_point_rel_gate_0
        frame = "gate_0"
    elif (drone_interface.drone_id == drones_ns[1]):
        point = initial_point_rel_gate_1
        frame = "gate_1"
    drone_interface.go_to.go_to_point_path_facing(
        point=point, speed=speed, frame_id=frame
    )


def confirm(uavs: List[DroneInterface], msg: str = 'Continue') -> bool:
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    elif confirmation == "n":
        return False
    else:
        shutdown_all(uavs)


def run_func(uavs: List[DroneInterface], func, *args):
    threads = []
    for uav in uavs:
        t = threading.Thread(target=func, args=(uav, *args))
        threads.append(t)
        t.start()
    print("Waiting for threads to finish...")
    for t in threads:
        t.join()
    print("all done")


def follow_path_with_go_to_uavs(uavs):
    run_func(uavs, follow_path_with_go_to)
    return


def initial_go_to_uavs(uavs):
    run_func(uavs, initial_go_to)
    return


def print_status(drone_interface: DroneInterface):
    while (True):
        drone_interface.get_logger().info(str(drone_interface.go_to.status))

if __name__ == '__main__':

    rclpy.init()
    uavs = []
    for ns in drones_ns:
        uavs.append(DroneInterface(ns, verbose=False, use_sim_time=parser.parse_args().simulated))

    print("Takeoff")
    if confirm(uavs, "Takeoff"):
        run_func(uavs, takeoff)
    print("Initial Go To")
    if confirm(uavs, "Go To"):
        initial_go_to_uavs(uavs)
    print("Follow Path")
    if confirm(uavs, "Follow Path"):
        follow_path_with_go_to_uavs(uavs)
    while confirm(uavs, "Replay"):
        follow_path_with_go_to_uavs(uavs)

    print("Land")
    if confirm(uavs, "Land"):
        run_func(uavs, land)

    print("Shutdown")
    rclpy.shutdown()

    exit(0)
