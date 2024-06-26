#!/bin/python3

from time import sleep
import argparse
import rclpy
from as2_python_api.drone_interface import DroneInterface


def drone_run(drone_interface: DroneInterface):
    """ Run the mission """

    speed = 0.5
    takeoff_height = 1.0
    height = 1.0

    sleep_time = 2.0

    dim = 1.0
    path = [
        [-dim, dim, height],
        [-dim, -dim, height],
        [dim, -dim, height],
        [dim, dim, height]
    ]
    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")
    # sleep(sleep_time)

    ##### GO TO #####
    while True:
        drone_interface.go_to.go_to_point([2.0, 0.0, 1.0], speed=0.5)
        print("Flying to 2.0 x")
        sleep(10)
        drone_interface.go_to.go_to_point([-2.0, 0.0, 1.0], speed=0.5)
        print("Flying to -2.0 x")
        sleep(10)

    # Example using 
    # path = [
    #     [0.3, 0.0, 2.0],
    #     [-2.0, 0.0, 1.2],
    #     [2.0, 2.0, 1.5],
    #     [2.5, 1.5, 0.8],
    #     [1.0, 0.0, 1.0],
    # ]
    # drone_interface.follow_path.follow_path_with_keep_yaw(path, speed=1.0)
    # for goal in path:
    #     print(f"Go to with path facing {goal}")
    #     drone_interface.go_to.go_to_point_path_facing(goal, speed=speed)
    #     print("Go to done")
    # sleep(sleep_time)

    # ##### LAND #####
    # print("Landing")
    # drone_interface.land(speed=0.5)
    # print("Land done")

    # drone_interface.disarm()


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
