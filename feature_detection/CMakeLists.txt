cmake_minimum_required(VERSION 2.8.3)
project(feature_detection)

find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rostime)

catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp rostime
  DEPENDS system_lib
)

add_executable(feature_detection_node src/feature_detection.cpp)

target_link_libraries(feature_detection_node ${catkin_LIBRARIES})

install(TARGETS feature_detection_node 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
