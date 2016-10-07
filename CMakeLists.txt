cmake_minimum_required(VERSION 2.8.3)
project(hexapod)
add_definitions(-std=c++11)
set(serial_DIR /tmp/usr/local/share/serial/cmake)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
    sensor_msgs
    geometry_msgs
)

include_directories( 
  ${catkin_INCLUDE_DIRS}
)


add_executable(serial_node src/serial_node.cpp)
add_executable(main_node src/main_node.cpp)

target_link_libraries(serial_node  ${catkin_LIBRARIES})
target_link_libraries(main_node ${catkin_LIBRARIES})
