#!/bin/bash

# PCL — Point Cloud Library (C++ native, fast on Jetson)
sudo apt install libpcl-dev -y
sudo apt install ros-humble-pcl-ros -y
sudo apt install ros-humble-pcl-conversions -y

# OpenCV — for depth image filtering (bilateral, inpainting)
sudo apt install libopencv-dev -y
sudo apt install ros-humble-cv-bridge -y

# Eigen3 — linear algebra (used by PCL internally, also useful directly)
sudo apt install libeigen3-dev -y

# Message filters — for time-synchronizing topics in C++
sudo apt install ros-humble-message-filters -y
