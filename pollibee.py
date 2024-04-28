#!/bin/python3

from time import sleep
import argparse
import rclpy
import json
import numpy as np
from typing import Dict,List

# from pollibee_src.utils import find_point, distance_to_collision

from as2_python_api.drone_interface import DroneInterface

def rotation_matrix(roll, pitch, yaw):
    """
    Calculate rotation matrix from roll, pitch, and yaw angles.
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])

    return np.dot(R_z, np.dot(R_y, R_x))


# def avoid_obstacle_path(initial_path: List[List[float]], flowers:Dict):
#     # See https://github.com/sarthak268/Obstacle_Avoidance_SUAS_2018/blob/master/main.py
#     safety_distance=0.2

#     obstacle = [f["xyz"][:-1] for f in flowers]

#     waypoint_final = []
#     for i, wp in range(len(initial_path)):

#         if (i == 0):
#             waypoint_final.append(initial_path[i])

#         safe = True
#         for j in range(len(obstacle)):
#             d = distance_to_collision(initial_path[i-1],initial_path[i], obstacle[j][:-1])
#             r = obstacle[j][2]
#             if (d < r + safety_distance):
#                 safe = False
#                 #print ("thukka")
#                 break
#         if (safe):
#             waypoint_final.append(initial_path[i])
#             #print ("safe tha")
#         else:
#             for l in range(len(obstacle)):
#                 d = distance_to_collision(initial_path[i-1],initial_path[i],obstacle[l][:-1])
#                 r = obstacle[l][2]
#                 if (d < r + safety_distance):
#                     ##################################################
#                     # finding the point to be added to waypoint file 
#                     # using intersection of two tangents from both the 
#                     # waypoints.
#                     a, b = find_point(initial_path[i-1],initial_path[i],r + safety_distance,[obstacle[l][0], obstacle[l][1]])
#                     ##################################################
#                     intersection = [a,b]
#                     waypoint_final.append(intersection)

#             waypoint_final.append(initial_path[i])
# 	#print (waypoint_final)



def fly_approach_flower(drone_interface: DroneInterface, flower:Dict, flowers: Dict):
    # This will work by first flying directly to a location perpendicular to the object a set distance away
    # The drone will then slowly fly into the object, move back and then fly to the next location
    # Convert object pose to separate variables
    distance_approach = 1.0  # Distance to first waypoint (50cm)
    distance_stop = 0.2  # Distance to stop (10cm)
    transit_speed = 1.0
    inspection_speed = 0.2

    x_obj, y_obj, z_obj = flower["xyz"]
    roll_obj, pitch_obj, yaw_obj = flower["rpy"]

    # Calculate direction vector towards the object
    direction_vector = -np.array([ np.cos(yaw_obj) * np.cos(pitch_obj),
                                  np.sin(yaw_obj) * np.cos(pitch_obj),
                                  -np.sin(pitch_obj),
                                 ])
    
    # Apply object's orientation to the direction vector
    R = rotation_matrix(roll_obj, pitch_obj, yaw_obj)
    direction_vector = np.dot(R, direction_vector)

    # Calculate the position of the first waypoint
    first_waypoint = np.array([x_obj, y_obj, z_obj]) + distance_approach * direction_vector

    # Calculate the position of the second waypoint (stop point)
    second_waypoint = np.array([x_obj, y_obj, z_obj]) + distance_stop * direction_vector

    # Calculate yaw angle for each waypoint
    yaw_angles = []
    for waypoint in [first_waypoint, second_waypoint]:
        delta = waypoint - np.array([x_obj, y_obj, z_obj])
        yaw_angle = np.arctan2(delta[1], delta[0]) + np.pi
        yaw_angles.append(yaw_angle)

    # With PID
    drone_interface.go_to.go_to_point_path_facing(first_waypoint, speed=1.0)
    drone_interface.go_to.go_to_point_with_yaw(first_waypoint, angle=yaw_angles[0], speed=transit_speed)
    sleep(1)
    print("step 1")
    drone_interface.go_to.go_to_point_with_yaw(second_waypoint, angle=yaw_angles[1], speed=inspection_speed)
    sleep(1)
    print("step 2")
    drone_interface.go_to.go_to_point_with_yaw(first_waypoint, angle=yaw_angles[0], speed=inspection_speed)
    print("step 3")

    # With Differential Flatness as Trajecotries
    # drone_interface.follow_path.follow_path_with_yaw([drone_interface.position(), first_waypoint], angle=yaw_angles[0], speed=tran)


def drone_run(drone_interface: DroneInterface, world_json_path: str):
    """ Run the mission """

    takeoff_height = 1.0
    sleep_time = 2.0

    with open(world_json_path, 'r') as f:
        world = json.load(f)
        objects = world["objects"]
    
    print("Start mission")

    ##### ARM OFFBOARD #####
    print("Arm")
    drone_interface.offboard()
    # sleep(sleep_time)
    print("Offboard")
    drone_interface.arm()
    # sleep(sleep_time)

    ##### TAKE OFF #####
    print("Take Off")
    drone_interface.takeoff(takeoff_height, speed=1.0)
    print("Take Off done")

    ##### GO TO #####
    for _ in range(3):
        for obj in objects:
            print(f'Flying To {obj["model_name"]}: {obj["xyz"]}')
            fly_approach_flower(drone_interface, obj, objects)
            sleep(sleep_time)

    print("Visited all objects complete")

    # ##### LAND #####
    drone_interface.go_to.go_to_with_yaw(0.0, 0.0, 1.0, angle=0.0, speed=1.0)

    print("Landing")
    drone_interface.land(speed=0.5)
    print("Land done")

    drone_interface.disarm()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(
        description="Starts gates mission for crazyswarm in either simulation or real environment")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)
    parser.add_argument('-w', '--world_file', default="sim_config/world.json")

    args = parser.parse_args()

    if args.simulated:
        print("Mission running in simulation mode")
    else:
        print("Mission running in real mode")

    rclpy.init()

    uav = DroneInterface(drone_id="cf0", verbose=False,
                         use_sim_time=args.simulated)

    drone_run(uav, args.world_file)

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
