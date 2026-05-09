#!/bin/bash


# terminal_1      realSense D435 camera
#
source /opt/ros/humble/setup.bash
source ~/robocon_ws/install/setup.bash

ros2 run realsense2_camera realsense2_camera_node \
  --ros-args \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p enable_sync:=false \
  -p initial_reset:=true \
  -p align_depth.enable:=true \
  -p pointcloud__neon_.enable:=true \
  -p pointcloud__neon_.ordered_pc:=false \
  -p rgb_camera.color_profile:=640x480x30 \
  -p depth_module.depth_profile:=848x480x30 \
  -p decimation_filter.enable:=true \
  -p decimation_filter.filter_magnitude:=2 \
  -p spatial_filter.enable:=true \
  -p spatial_filter.filter_magnitude:=2 \
  -p spatial_filter.filter_smooth_alpha:=0.5 \
  -p spatial_filter.filter_smooth_delta:=20.0 \
  -p temporal_filter.enable:=true \
  -p temporal_filter.filter_smooth_alpha:=0.4 \
  -p temporal_filter.filter_smooth_delta:=20.0 \
  -p hole_filling_filter.enable:=true \
  -p hole_filling_filter.holes_fill:=2 \
  -p clip_distance:=4.0 \
  -p enable_rgbd:=true
