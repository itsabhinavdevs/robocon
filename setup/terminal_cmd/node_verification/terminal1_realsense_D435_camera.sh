#!/bin/bash

ros2 topic list | grep camera
# must see /camera/camera/depth/color/points in the list

ros2 topic hz /camera/camera/color/image_raw       #  ~30 Hz
ros2 topic hz /camera/camera/depth/color/points    #  ~15-30 Hz
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw  #  ~30 Hz
