cmake_minimum_required(VERSION 3.0.2)
project(livox_cam_tools)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -w -O2 ")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  pcl_ros
  roscpp
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES livox_cam_tools
#  CATKIN_DEPENDS cv_bridge image_transport pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_subdirectory(include)
#------------image recorder-----------------
add_executable(img_recorder src/img_recorder.cc)

target_link_libraries(img_recorder
  ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
)

#------------livox_map_simu-----------------
add_executable(livox_map_simu src/livox_map_simu.cc)

target_link_libraries(livox_map_simu
  ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
)

#------------livox_map_simu-----------------
add_executable(livox_map_real src/livox_map_real.cc)

target_link_libraries(livox_map_real
  ${catkin_LIBRARIES}
  # ${PCL_LIBRARIES}
)

#------------bag_extractor-----------------
add_executable(bag_extractor src/bag_extractor.cc)

target_link_libraries(bag_extractor
  ${catkin_LIBRARIES}
  lib_bagreader
)

#------------bag_extractor-----------------
add_executable(liv_map_cam_recorder src/liv_map_cam_recorder.cc)

target_link_libraries(liv_map_cam_recorder
  ${catkin_LIBRARIES}
)
#------------rotm2eul-----------------
add_executable(rotm2eul src/rotm2eul.cc)

target_link_libraries(rotm2eul
  ${catkin_LIBRARIES}
)