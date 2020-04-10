cmake_minimum_required(VERSION 2.8.3)
project(openzen_sensor)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
)

SET(ZEN_BLUETOOTH OFF CACHE BOOL "Don't build bluetooth")
SET(ZEN_TESTS OFF CACHE BOOL "Don't build tests")
SET(ZEN_EXAMPLES OFF CACHE BOOL "Don't build examples")
add_subdirectory(openzen)

## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(openzen_sensor_node
  src/OpenZenNode.cpp
)

target_link_libraries(openzen_sensor_node
  ${catkin_LIBRARIES}
  OpenZen
)

add_dependencies(openzen_sensor_node ${catkin_EXPORTED_TARGETS})
