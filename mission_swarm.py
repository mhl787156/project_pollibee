#!/bin/python3

import sys
import argparse
import rclpy
from as2_python_api.drone_interface import DroneInterface
from mission import run_mission, shutdown_all


def main():
    """ Main """
    parser = argparse.ArgumentParser(
        description="Starts gates mission for crazyswarm in either simulation or real environment")
    parser.add_argument('-s', '--simulated',
                        action='store_true', default=False)

    input_args = parser.parse_args()

    rclpy.init()

    drones_namespaces = ['cf0', 'cf1']
    drones = []
    for namespace in drones_namespaces:
        drones.append(
            DroneInterface(
                namespace,
                verbose=False,
                use_sim_time=input_args.simulated))

    # Gates
    gates_namespaces = ['gate_0', 'gate_1', 'gate_2']
    if input_args.simulated:
        print("Mission running in simulation mode")
        gates_heights = [2.0, 2.0]
    else:
        print("Mission running in real mode")
        gates_heights = [2.0, 2.0]
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
