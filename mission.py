#!/bin/python3

"""
mission_vel_motion_ref.py
"""

import time
import rclpy
import argparse
from as2_python_api.drone_interface import DroneInterface
from as2_python_api.modules.motion_reference_handler_module import MotionReferenceHandlerModule


class DroneMotionRef(DroneInterface):
    """Drone interface node with PointGimbalModule."""

    def __init__(self, name, verbose=False, use_sim_time=False):
        super().__init__(name, verbose, use_sim_time)

        self.motion_ref_handler = MotionReferenceHandlerModule(drone=self)

    def run_test(self):
        """ Run the mission """
        self.offboard()
        self.arm()

        # Takeoff to 1 meter
        self.takeoff(1.0, wait=True)

        time.sleep(1.0)

        while True:
            # Continuously send 0.5m/s in x direction with 0.0 yaw
            self.motion_ref_handler.speed.send_speed_command_with_yaw_angle(
                [0.5, 0.0, 0.0], pose=None, twist_frame_id=f'{self.drone_id}/base_link', yaw_angle=0.0)
            time.sleep(0.1)


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

    uav = DroneMotionRef("cf0", verbose=True, use_sim_time=True)
    uav.run_test()
    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)

