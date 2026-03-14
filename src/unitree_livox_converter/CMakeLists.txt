cmake_minimum_required(VERSION 3.0.2)
project(unitree_livox_converter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  livox_ros_driver
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud2_to_livox_custom
  src/pointcloud2_to_livox_custom.cpp
)

target_link_libraries(pointcloud2_to_livox_custom
  ${catkin_LIBRARIES}
)