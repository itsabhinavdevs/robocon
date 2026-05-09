#!/bin/bash


ros2 topic hz /camera/depth/filtered          #  ~30 Hz
ros2 topic hz /camera/pointcloud/filtered     #  ~15-30 Hz
ros2 topic hz /camera/pointcloud/ground       #  ~15-30 Hz
