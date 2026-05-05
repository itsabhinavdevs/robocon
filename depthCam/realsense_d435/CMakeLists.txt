# Comment
# Replace ~/robocon_ws/src/robocon_odometry/CMakeLists.txt     with this file


cmake_minimum_required(VERSION 3.8)
project(robocon_odometry)

# Use C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation)
find_package(Eigen3 REQUIRED)

# Common dependency list used by all targets
set(DEPS
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  message_filters
)

#     Node 1: depth_filter_node
add_executable(depth_filter_node src/depth_filter_node.cpp)
ament_target_dependencies(depth_filter_node ${DEPS})
target_link_libraries(depth_filter_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen)
target_include_directories(depth_filter_node PUBLIC
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

#     Node 2: pointcloud_filter_node
add_executable(pointcloud_filter_node src/pointcloud_filter_node.cpp)
ament_target_dependencies(pointcloud_filter_node ${DEPS})
target_link_libraries(pointcloud_filter_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  Eigen3::Eigen)
target_include_directories(pointcloud_filter_node PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

#     Node 3: distance_calculator
add_executable(distance_calculator src/distance_calculator.cpp)
ament_target_dependencies(distance_calculator ${DEPS})
target_link_libraries(distance_calculator Eigen3::Eigen)
target_include_directories(distance_calculator PUBLIC include)

#     Install targets
install(TARGETS
  depth_filter_node
  pointcloud_filter_node
  distance_calculator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
