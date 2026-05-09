#!/bin/bash


# terminal 2 — depth and point cloud filter nodes (run after Terminal 1 is stable)

source /opt/ros/humble/setup.bash
source ~/robocon_ws/install/setup.bash

# launch depth image filter in background
ros2 run robocon_odometry depth_filter_node \
  --ros-args \
  -p min_depth_mm:=300 \
  -p max_depth_mm:=4000 \
  -p bilateral_d:=5 \
  -p bilateral_sigma:=50.0 \
  -p edge_kernel_size:=3 \
  -p temporal_frames:=5 \
  -p enable_temporal:=true \
  -p enable_bilateral:=true \
  -p enable_edge_removal:=true \
  -p enable_inpaint:=true &

# launch point cloud filter in foreground
ros2 run robocon_odometry pointcloud_filter_node \
  --ros-args \
  -p voxel_size:=0.05 \
  -p min_depth:=0.3 \
  -p max_depth:=4.0 \
  -p min_height:=-0.5 \
  -p max_height:=2.0 \
  -p sor_k_neighbors:=20 \
  -p sor_std_ratio:=2.0 \
  -p enable_sor:=true \
  -p enable_radius_filter:=false \
  -p remove_ground:=true \
  -p ground_dist_thresh:=0.02 \
  -p ground_ransac_iters:=500 \
  -p estimate_normals:=true \
  -p normal_radius:=0.10
