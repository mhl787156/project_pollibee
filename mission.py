#!/bin/python3

import sys
import argparse
import time
import threading
from typing import List
import rclpy
from as2_python_api.drone_interface import DroneInterface

SPEED = 1.0  # Speed of the drone
INGORE_YAW = True  # Yaw mode


def shutdown_all(uavs):
    """ Shutdown all uavs """
    print("Exiting...")
    for uav in uavs:
        uav.shutdown()
    sys.exit(1)


def takeoff(uav: DroneInterface):
    """ Takeoff the drone """
    uav.arm()
    uav.offboard()
    uav.takeoff(2.0, 0.7)
    time.sleep(1)


def land(drone_interface: DroneInterface):
    """ Land the drone """
    drone_interface.land(0.5)


def go_to(drone_interface: DroneInterface, go_to_point: List, frame_id: str):
    """ Go to a point """
    drone_interface.go_to.go_to_point_path_facing(
        point=go_to_point, speed=SPEED, frame_id=frame_id)


def follow_path(drone_interface: DroneInterface, path: List, frame_id: str):
    """ Follow a path """
    drone_interface.follow_path.follow_path_with_path_facing(
        path, speed=SPEED, frame_id=frame_id)


def initial_go_to_drones(drone_interface: DroneInterface, drones_namespaces_list: List, paths: List):
    """ Initial go to for all drones """
    # Get index of drone interface in list of drone interfaces
    drone_index = drones_namespaces_list.index(drone_interface.drone_id)
    path_dict = paths[drone_index]
    path_frame_id = list(path_dict.keys())[0]
    path_value = paths[drone_index][path_frame_id][0]

    go_to(drone_interface, path_value, frame_id=path_frame_id)


def follow_path_drones(drone_interface: DroneInterface, drones_namespaces_list: List, paths: List):
    """ Follow path for all drones """
    drone_index = drones_namespaces_list.index(drone_interface.drone_id)

    # Sort paths from drone_index to the end and from the beginning to drone_index
    paths_to_do = []
    for i in range(drone_index, len(paths)):
        paths_to_do.append(paths[i])
    for i in range(0, drone_index):
        paths_to_do.append(paths[i])

    for path in paths_to_do:
        path_frame_id = list(path.keys())[0]
        path_value = path[path_frame_id]
        follow_path(drone_interface, path_value, frame_id=path_frame_id)
        # for waypoint in path_value:
        #     go_to(drone_interface, waypoint, frame_id=path_frame_id)


def confirm(msg: str = 'Continue') -> bool:
    """ Ask for confirmation """
    confirmation = input(f"{msg}? (y/n): ")
    if confirmation == "y":
        return True
    return False


def run_func(drones_list: List[DroneInterface], func, *args):
    """ Run a function in parallel """
    threads = []
    for drone in drones_list:
        thread = threading.Thread(target=func, args=(drone, *args))
        threads.append(thread)
        thread.start()
    print("Waiting for threads to finish...")
    for thread in threads:
        thread.join()
    print("All done")


def get_paths(
        gates_namespaces: List,
        gates_heights: List,
        gates_desp_x: List,
        gates_desp_y: List):
    """ Get paths for all drones """

    paths = []
    for index, (gate_ns, gate_height, gate_desp_x, gate_desp_y) in enumerate(
            zip(gates_namespaces, gates_heights, gates_desp_x, gates_desp_y)):
        path = []
        gate_points = [
            [gate_desp_x, gate_desp_y, gate_height],  # Before
            [0.0, 0.0, gate_height],                  # Center
            [-gate_desp_x, gate_desp_y, gate_height],  # After
        ]

        if gate_desp_x != 0.0 or gate_desp_y != 0.0:
            path = [gate_points[0], gate_points[1], gate_points[2]]
        else:
            path = [gate_points[1]]

        paths.append({gate_ns: path})
    return paths


def run_mission(
        drones_interfaces,
        gates_namespaces,
        gates_heights,
        gates_desp_x,
        gates_desp_y):
    """ Run the mission """

    if len(drones_interfaces) > len(gates_namespaces):
        print("Number of drones must be less or equal than number of gates")
        return

    if len(gates_namespaces) != len(gates_heights) != len(gates_desp_x) != len(gates_desp_y):
        print("Number of gates must be equal to number of heights, desp_x and desp_y")
        return

    paths = get_paths(
        gates_namespaces,
        gates_heights,
        gates_desp_x,
        gates_desp_y)

    drones_namespaces = []
    for drones_interface in drones_interfaces:
        drones_namespaces.append(drones_interface.drone_id)

    print("Takeoff")
    if confirm("Takeoff"):
        run_func(drones_interfaces, takeoff)
        print("Take Off done")

        if confirm("Go To Initial"):
            run_func(drones_interfaces, initial_go_to_drones,
                     drones_namespaces, paths)
            if confirm("Follow Path"):
                run_func(drones_interfaces, follow_path_drones,
                         drones_namespaces, paths)
                print("Path done")
                while confirm("Replay"):
                    run_func(drones_interfaces, follow_path_drones,
                             drones_namespaces, paths)
                    print("Path done")

        print("Land")
        if confirm("Land"):
            run_func(drones_interfaces, land)


def main():
    """ Main """

    parser = argparse.ArgumentParser(
        description="Starts gates mission for crazyswarm in either simulation or real environment")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)

    input_args = parser.parse_args()

    rclpy.init()

    drones_namespaces = ['cf0']
    drones = []
    for namespace in drones_namespaces:
        drones.append(
            DroneInterface(
                namespace,
                verbose=False,
                use_sim_time=input_args.simulated))

    # Gates
    gates_namespaces = ['gate_0/link', 'gate_1/link']
    if input_args.simulated:
        print("Mission running in simulation mode")
        gates_heights = [2.0, 2.0]
    else:
        print("Mission running in real mode")
        gates_heights = [0.8, 0.8]
    gates_desp_x = [1.0, 1.0]
    gates_desp_y = [0.0, 0.0]

    # Run mission
    run_mission(
        drones,
        gates_namespaces,
        gates_heights,
        gates_desp_x,
        gates_desp_y)

    print("Shutdown")
    shutdown_all(drones)
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
