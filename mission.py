#!/bin/python3

import os
from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import YawMode


def drone_run(drone_interface: DroneInterface):

    speed = 2.0
    takeoff_height = 0.5
    height = 2.0

    sleep_time = 4.0
    yaw_mode = YawMode()
    yaw_mode.mode = YawMode.PATH_FACING

    dim = 2.0
    path = [
        [dim, dim, height],
        [dim, -dim, height],
        [-dim, dim, height],
        [-dim, -dim, height],
        [0.0, 0.0, takeoff_height],
    ]

    print("Start mission")

    ##### ARM OFFBOARD #####
    drone_interface.offboard()
    drone_interface.arm()

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=0.2)
    print("Take Off done")
    sleep(sleep_time)

    ##### FOLLOW PATH #####
    sleep(sleep_time)
    print(f"Follow path with path facing: [{path}]")
    drone_interface.follow_path.follow_path_with_path_facing(path, speed)
    print("Follow path done")

    sleep(sleep_time)
    print(f"Follow path with keep yaw: [{path}]")
    drone_interface.follow_path.follow_path_with_keep_yaw(path, speed)
    print("Follow path done")

    sleep(sleep_time)
    print(f"Follow path with angle {-1.57}: [{path}]")
    drone_interface.follow_path.follow_path_with_yaw(path, speed, angle=-1.57)
    print("Follow path done")

    ##### GOTO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        drone_interface.goto.go_to_point_path_facing(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    for goal in path:
        print(f"Go to with keep yaw {goal}")
        drone_interface.goto.go_to_point(goal, speed=speed)
        print("Go to done")
    sleep(sleep_time)

    for goal in path:
        print(f"Go to with angle {-1.57}: {goal}")
        drone_interface.goto.go_to_point_with_yaw(goal, speed=speed, angle=-1.57)
        print("Go to done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    drone_interface.land(speed=0.3)
    print("Land done")


if __name__ == '__main__':
    rclpy.init()
    # Get environment variable AEROSTACK2_SIMULATION_DRONE_ID
    uav_name = "cf"
    uav = DroneInterface(uav_name, verbose=False, use_sim_time=True)

    drone_run(uav)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
