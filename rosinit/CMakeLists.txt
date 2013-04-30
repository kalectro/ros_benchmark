cmake_minimum_required(VERSION 2.8.3)
project(rosinit)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
)

add_executable(rosinit src/rosinit_benchmark.cpp)

target_link_libraries(rosinit ${catkin_LIBRARIES})

install(TARGETS rosinit 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
