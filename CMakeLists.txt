cmake_minimum_required(VERSION 2.6)

project(hls_lfom_tof_driver)

set(CMAKE_C_COMPILER "gcc")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-std=gnu++11")
set(CMAKE_CXX_FLAGS_DEBUG "-std=gnu++11 -g -O0")

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(hlds_3dtof_node src/hlds_3dtof_node.cpp)

add_dependencies(
  hlds_3dtof_node 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  hlds_3dtof_node
  /usr/lib/hldstof/libtof.a
  rt m pthread stdc++
  ${catkin_LIBRARIES}
)