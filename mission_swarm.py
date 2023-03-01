#!/bin/python3

import sys
import time
import threading
from typing import List
import math
import rclpy
from gates import Gates
from tf2_ros.buffer_interface import TransformRegistration
from tf2_geometry_msgs import PointStamped, TransformStamped
from as2_python_api.drone_interface import DroneInterface

# pos= [[1,0,1],[-1,1,1.5],[-1,-1,2.0]]

speed = 2.5
ingore_yaw = True
height = 2.2
desp_gates = 0.5


drones_ns = [
    'cf0', 'cf1']

drone_turn = 0

# gate_1_pose = PoseStamped()
# gate_2_pose = PoseStamped()

t_gate_0 = TransformStamped()
t_gate_1 = TransformStamped()

# position_gate_0 = [-1.5, 1.0, 1.9, 0.0, 0.0, 0.0, 0.0]
# position_gate_1 = [2.5, 1.0, 1.9, 0.0, 0.0, 0.0, 0.0]
gates_node = Gates()

rclpy.spin_once(gates_node)
position_gate_0 = gates_node.get_gate_0_pose()
position_gate_1 = gates_node.get_gate_1_pose()

# position_gate_0[2] = 2.0
# position_gate_1[2] = 2.0

position_gate_0[2] += 0.6
position_gate_1[2] += 0.7

print(position_gate_0)
print(position_gate_1)

gates_node.shutdown()

h_dist = math.sqrt((position_gate_0[0] - position_gate_1[0])
                   ** 2 + (position_gate_0[1] - position_gate_1[1])**2)

v_dist = 4.0

initial_point_rel_gate_0 = [h_dist/2,
                            -v_dist/2, 0.0]

initial_point_rel_gate_1 = [-h_dist/2,
                            v_dist/2, 0.0]
if desp_gates != 0.0:

    poses_rel_gate_0 = [[0.0, -desp_gates,
                        0.0], [0.0, desp_gates, 0.0]]
    poses_rel_gate_1 = [[0.0, desp_gates,
                        0.0], [0.0, -desp_gates, 0.0]]
else:

    poses_rel_gate_0 = [[0.0, 0.0, 0.0]]
    poses_rel_gate_1 = [[0.0, 0.0, 0.0]]


path_gate_0 = []
path_gate_1 = []


def gates_transforms():
    global t_gate_0, t_gate_1

    t_gate_0.header.frame_id = 'earth'
    t_gate_0.child_frame_id = 'gate_0'
    t_gate_0.transform.translation.x = position_gate_0[0]
    t_gate_0.transform.translation.y = position_gate_0[1]
    t_gate_0.transform.translation.z = position_gate_0[2]
    t_gate_0.transform.rotation.x = 0.0
    t_gate_0.transform.rotation.y = 0.0
    t_gate_0.transform.rotation.z = position_gate_0[5]
    t_gate_0.transform.rotation.w = position_gate_0[6]

    t_gate_1.header.frame_id = 'earth'
    t_gate_1.child_frame_id = 'gate_1'
    t_gate_1.transform.translation.x = position_gate_1[0]
    t_gate_1.transform.translation.y = position_gate_1[1]
    t_gate_1.transform.translation.z = position_gate_1[2]
    t_gate_1.transform.rotation.x = 0.0
    t_gate_1.transform.rotation.y = 0.0
    t_gate_1.transform.rotation.z = position_gate_1[5]
    t_gate_1.transform.rotation.w = position_gate_1[6]


def list_to_point(_l: list):
    ret = PointStamped()
    ret.point.x = _l[0]
    ret.point.y = _l[1]
    ret.point.z = _l[2]

    return ret


def point_to_list(point: PointStamped):
    ret = []
    ret.append(point.point.x)
    ret.append(point.point.y)
    ret.append(point.point.z)

    return ret


def transform_waypoints_from_gates_to_earth():
    global initial_point_rel_gate_0, initial_point_rel_gate_1

    registration = TransformRegistration()
    do_transform = registration.get(PointStamped)

    initial_point_rel_gate_0 = point_to_list(do_transform(
        list_to_point(initial_point_rel_gate_0), t_gate_0))
    initial_point_rel_gate_1 = point_to_list(do_transform(
        list_to_point(initial_point_rel_gate_1), t_gate_1))

    # path_gate_0.append(initial_point_rel_gate_0)
    # path_gate_1.append(initial_point_rel_gate_1)

    for point in poses_rel_gate_0:
        path_gate_0.append(point_to_list(
            do_transform(list_to_point(point), t_gate_0)))

    path_gate_0.append(initial_point_rel_gate_1)

    for point in poses_rel_gate_1:
        path_gate_1.append(point_to_list(
            do_transform(list_to_point(point), t_gate_1)))

    path_gate_1.append(initial_point_rel_gate_0)

    print(f"Path 0: {path_gate_0}")
    print(f"Path 1: {path_gate_1}")


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


def follow_path(drone_interface: DroneInterface):
    path = []
    if drone_interface.drone_id == drones_ns[0]:
        if drone_turn == 0:
            path = path_gate_0
        else:
            path = path_gate_1
    elif drone_interface.drone_id == drones_ns[1]:
        if drone_turn == 0:
            path = path_gate_1
        else:
            path = path_gate_0

    drone_interface.follow_path.follow_path_with_keep_yaw(
        path=path, speed=speed)
    # drone_interface.goto.go_to_point_path_facing(o
    #     pose_generator(drone_interface), speed=speed)


def go_to(drone_interface: DroneInterface):
    if (drone_interface.drone_id == drones_ns[0]):
        point = initial_point_rel_gate_0
    elif (drone_interface.drone_id == drones_ns[1]):
        point = initial_point_rel_gate_1
    drone_interface.goto.go_to_point(
        point=point, speed=speed
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


def follow_path_uavs(uavs):
    run_func(uavs, follow_path)
    return


def go_to_uavs(uavs):
    run_func(uavs, go_to)
    return


def print_status(drone_interface: DroneInterface):
    while (True):
        drone_interface.get_logger().info(str(drone_interface.goto.status))


if __name__ == '__main__':

    # rclpy.init()
    uavs = []
    for ns in drones_ns:
        uavs.append(DroneInterface(ns, verbose=False))

    print("Initial transformations")
    gates_transforms()
    transform_waypoints_from_gates_to_earth()
    print("Takeoff")
    if confirm(uavs, "Takeoff"):
        run_func(uavs, takeoff)
    print("Initial Go To")
    if confirm(uavs, "Go To"):
        go_to_uavs(uavs)
    print("Follow Path")
    if confirm(uavs, "Follow Path"):
        follow_path_uavs(uavs)
    drone_turn = 1
    while confirm(uavs, "Replay"):
        follow_path_uavs(uavs)
        drone_turn = abs(drone_turn) - 1

    print("Land")
    if confirm(uavs, "Land"):
        run_func(uavs, land)

    print("Shutdown")
    rclpy.shutdown()

    exit(0)
