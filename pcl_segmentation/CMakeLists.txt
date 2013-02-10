cmake_minimum_required(VERSION 2.8.3)
project(pcl_segmentation)

find_package(catkin REQUIRED COMPONENTS pcl pcl_ros roscpp sensor_msgs)

catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES pcl_segmentation
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_segmentation src/segment.cpp)

target_link_libraries(pcl_segmentation ${catkin_LIBRARIES})
