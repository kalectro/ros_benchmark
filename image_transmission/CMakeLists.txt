cmake_minimum_required(VERSION 2.8.3)
project(image_transmission)

find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport std_msgs roscpp)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(image_transmitter src/image_transport_publisher.cpp)
add_executable(image_subscriber src/image_transport_subscriber.cpp)

target_link_libraries(image_transmitter ${catkin_LIBRARIES})
target_link_libraries(image_subscriber ${catkin_LIBRARIES})

install(TARGETS image_transmitter image_subscriber 
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
        
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY images DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
