cmake_minimum_required(VERSION 2.6)

project(hls_lfom_tof_driver)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_ros
  roscpp
  sensor_msgs
)

add_service_files(
  FILES
  FarSignalCutoff.srv
  LowSignalCutoff.srv
  DistanceMode.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  LIBRARIES hls_lfom_tof_driver
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