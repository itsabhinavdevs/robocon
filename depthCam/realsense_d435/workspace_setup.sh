#!/bin/bash

mkdir -p ~/robocon_ws/src
cd ~/robocon_ws/src

# Create C++ package (for cpp ament_cmake , for python ament_python)
ros2 pkg create --build-type ament_cmake robocon_odometry \
  --dependencies \
    rclcpp \
    sensor_msgs \
    nav_msgs \
    geometry_msgs \
    std_msgs \
    tf2_ros \
    tf2_geometry_msgs \
    cv_bridge \
    image_transport \
    pcl_ros \
    pcl_conversions \
    message_filters

# Create folder structure
mkdir -p ~/robocon_ws/src/robocon_odometry/src
mkdir -p ~/robocon_ws/src/robocon_odometry/include/robocon_odometry
mkdir -p ~/robocon_ws/src/robocon_odometry/config
mkdir -p ~/robocon_ws/src/robocon_odometry/launch

cd ~/robocon_ws
colcon build
echo "source ~/robocon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
