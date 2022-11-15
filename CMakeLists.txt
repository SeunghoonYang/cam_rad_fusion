cmake_minimum_required(VERSION 3.0.2)
project(cam_rad_fusion)

# add_compile_options(-std=c++11)

set (pcl_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  pcl
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cam_rad_fusion
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport pcl pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fusion src/main.cpp)
target_link_libraries(fusion ${catkin_LIBRARIES})