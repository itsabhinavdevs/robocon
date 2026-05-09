#!/bin/bash



# TERMINAL 3 — RTAB-Map (run after Terminal 2 is stable)

source /opt/ros/humble/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/depth/filtered \
  camera_info_topic:=/camera/camera/aligned_depth_to_color/camera_info \
  frame_id:=base_link \
  odom_frame_id:=odom \
  map_frame_id:=map \
  approx_sync:=true \
  approx_sync_max_interval:=0.05 \
  qos:=2 \
  rviz:=false \
  rtabmapviz:=false \
  visual_odometry:=true \
  icp_odometry:=false \
  publish_tf_odom:=true \
  odom_topic:=/visual_odometry \
  queue_size:=30 \
  Odom/Strategy:=0 \
  Odom/GuessMotion:=true \
  Odom/ResetCountdown:=1 \
  Vis/FeatureType:=6 \
  Vis/MaxFeatures:=400 \
  Vis/MinInliers:=15 \
  Vis/CorType:=0 \
  Reg/Force3DoF:=true \
  Mem/STMSize:=20 \
  Rtabmap/MemoryThr:=300
