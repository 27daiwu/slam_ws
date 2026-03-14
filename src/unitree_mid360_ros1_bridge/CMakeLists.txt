cmake_minimum_required(VERSION 3.0.2)
project(unitree_mid360_ros1_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
)

set(CycloneDDS_DIR "/home/unitree/cyclonedds_ws/install/cyclonedds/lib/cmake/CycloneDDS")
set(CycloneDDS-CXX_DIR "/home/unitree/cyclonedds_ws/install/cyclonedds_cxx/lib/cmake/CycloneDDS-CXX")

find_package(CycloneDDS REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
  /opt/unitree_robotics/include
  /home/unitree/cyclonedds_ws/install/cyclonedds_cxx/include/ddscxx
)

link_directories(
  /usr/local/lib
  /opt/unitree_robotics/lib
)

add_executable(mid360_dds_to_ros1_node
  src/mid360_dds_to_ros1_node.cpp
)

target_link_libraries(mid360_dds_to_ros1_node
  ${catkin_LIBRARIES}
  unitree_sdk2
  CycloneDDS::ddsc
  CycloneDDS-CXX::ddscxx
)

install(TARGETS mid360_dds_to_ros1_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)