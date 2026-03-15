cmake_minimum_required(VERSION 3.0.2)
project(pcd_to_2d_grid_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_to_2d_grid_map_node src/pcd_to_2d_grid_map_node.cpp)

target_link_libraries(pcd_to_2d_grid_map_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)