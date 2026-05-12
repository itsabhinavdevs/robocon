#!/bin/bash


cd ~/robocon_ws

# full build (first time, compiles all 3 nodes)
colcon build --packages-select robocon_odometry \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# source workspace
source install/setup.bash

# verify executables exist:
ls install/robocon_odometry/lib/robocon_odometry/
# should show: depth_filter_node  pointcloud_filter_node
