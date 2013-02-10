cmake_minimum_required(VERSION 2.8.3)
project(image_transmission)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport std_msgs roscpp)

catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport std_msgs roscpp
  DEPENDS system_lib
)

add_executable(image_transmitter src/image_transport_publisher.cpp)
add_executable(image_subscriber src/image_transport_subscriber.cpp)

target_link_libraries(image_transmitter ${catkin_LIBRARIES})
target_link_libraries(image_subscriber ${catkin_LIBRARIES})

