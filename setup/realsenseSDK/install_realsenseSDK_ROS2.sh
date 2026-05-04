#!/bin/bash
#
sudo apt install ros-humble-librealsense2* -y
sudo apt install ros-humble-realsense2-camera -y
sudo apt install ros-humble-realsense2-description -y
sudo apt install ros-humble-image-transport-plugins -y

# Verify
realsense-viewer
