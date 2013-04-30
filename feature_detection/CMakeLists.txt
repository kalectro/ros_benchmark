cmake_minimum_required(VERSION 2.8.3)
project(feature_detection)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rostime)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(feature_detection_node src/feature_detection.cpp)

target_link_libraries(feature_detection_node ${catkin_LIBRARIES})

install(TARGETS feature_detection_node 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
