cmake_minimum_required(VERSION 2.8.3)
project(pcl_segmentation)

find_package(catkin REQUIRED COMPONENTS pcl_ros roscpp sensor_msgs)
find_package(PCL 1.4.0 REQUIRED)

catkin_package(
)

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

add_executable(pcl_segmentation src/segmentation.cpp)
add_executable(create_pointcloud src/create_pointcloud.cpp)
add_executable(combined src/combined.cpp)

target_link_libraries(pcl_segmentation ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(create_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(combined ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS pcl_segmentation create_pointcloud combined
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
