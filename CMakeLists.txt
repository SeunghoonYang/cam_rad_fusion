cmake_minimum_required(VERSION 3.0.2)
project(cam_rad_fusion)

# add_compile_options(-std=c++11)

set (pcl_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")
find_package(catkin REQUIRED dynamic_reconfigure)
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
find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/dynamic_params.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES opencv
  CATKIN_DEPENDS dynamic_reconfigure
#  LIBRARIES cam_rad_fusion
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport pcl pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(fusion src/main.cpp)
target_link_libraries(fusion ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fusion ${PROJECT_NAME}_gencfg)

add_executable(dynamic_reconfigure src/main_dyn.cpp)
target_link_libraries(dynamic_reconfigure ${catkin_LIBRARIES})
add_dependencies(dynamic_reconfigure ${PROJECT_NAME}_gencfg)