#!/bin/bash


ros2 topic echo /visual_odometry --once
ros2 topic hz /visual_odometry     #  ~10-30 Hz
