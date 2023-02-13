#!/bin/bash

drone_namespace="cf"


mkdir rosbags 2>/dev/null
cd rosbags &&\
ros2 bag record \
"/aideck/image" \
"/$drone_namespace/actuator_command/pose" \
"/$drone_namespace/actuator_command/thrust" \
"/$drone_namespace/actuator_command/twist" \
"/$drone_namespace/controller/info" \
"/$drone_namespace/motion_reference/pose" \
"/$drone_namespace/motion_reference/twist" \
"/$drone_namespace/platform/info" \
"/$drone_namespace/self_localization/pose" \
"/$drone_namespace/self_localization/twist" \
"/$drone_namespace/sensor_measurements/battery" \
"/$drone_namespace/sensor_measurements/imu" \
"/$drone_namespace/sensor_measurements/odom" \
"/tf" \
"/tf_static" \
--qos-profile-overrides-path $(pwd)/../reliability_override.yaml --include-hidden-topics
