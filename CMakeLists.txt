cmake_minimum_required(VERSION 2.8.3)
project(openzen_sensor)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
)

option(OPENZEN_ROS_BINARY_DOWNLOAD "Download binary builds" OFF)
option(OPENZEN_ROS_BINARY_LIBRARIES "Download binary dependencies" OFF)

# check if we can build OpenZen or if we need to download
# a binary version
set(BUILD_OPENZEN ON)
if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
  message(WARNING "At least GCC version 7.0 needed to build OpenZen, will download binary version")
  set(BUILD_OPENZEN OFF)
endif()

if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
  message(WARNING "At least cmake version 3.10 needed to build OpenZen, will download binary version")
  set(BUILD_OPENZEN OFF)
endif()

if (NOT BUILD_OPENZEN AND NOT OPENZEN_ROS_BINARY_DOWNLOAD)
  message(FATAL_ERROR "Dependencies to build OpenZen not met and binary download disabled.")
endif()

if (BUILD_OPENZEN)
  SET(ZEN_BLUETOOTH OFF CACHE BOOL "Don't build bluetooth")
  SET(ZEN_TESTS OFF CACHE BOOL "Don't build tests")
  SET(ZEN_EXAMPLES OFF CACHE BOOL "Don't build examples")
  SET(ZEN_USE_BINARY_LIBRARIES ${OPENZEN_ROS_BINARY_LIBRARIES} CACHE BOOL "Don't download binary dependencies (windows only)")
  add_subdirectory(openzen)
  SET(OPENZEN_TARGET_NAME OpenZen)
else()
  set(OPENZEN_BINARY_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/OpenZenBinary")
  set(OPENZEN_ARCHIVE_FILE "${CMAKE_CURRENT_BINARY_DIR}/OpenZenRelease.tar.gz")
  set(OPENZEN_BASE_FOLDER "${CMAKE_CURRENT_BINARY_DIR}/openzen_release")

  if (CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set(OPENZEN_DOWNLOAD_URL "https://bitbucket.org/lpresearch/openzen/downloads/OpenZen-Ubuntu1604-x64-1.2.0.tar.gz")
    set(OPENZEN_DOWNLOAD_HASH 8a9f0ffa03fc1bd49fb41248a9c08c63e0f427086c579a2725105b0e0e0b768b)
  elseif((CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64") OR (CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "aarch64"))
    set(OPENZEN_DOWNLOAD_URL "https://bitbucket.org/lpresearch/openzen/downloads/OpenZen-Ubuntu1604-arm64-1.2.0.tar.gz")
    set(OPENZEN_DOWNLOAD_HASH 385f6751035dfba475efda8e90680921330b0c556ad6b8204ed626fd9ead4359)
  else()
    message(FATAL_ERROR "No binary OpenZen release available for platform ${CMAKE_HOST_SYSTEM_PROCESSOR}")
  endif()

  if(NOT EXISTS ${OPENZEN_BINARY_FOLDER})
    if(NOT EXISTS ${OPENZEN_ARCHIVE_FILE})
          message("Will unpack to ${OPENZEN_ARCHIVE_FILE}")
      message(STATUS "Downloading OpenZen release from ${OPENZEN_DOWNLOAD_URL}")
      file(DOWNLOAD ${OPENZEN_DOWNLOAD_URL} ${OPENZEN_ARCHIVE_FILE}
            SHOW_PROGRESS
            EXPECTED_HASH SHA256=${OPENZEN_DOWNLOAD_HASH})
    endif()

    # unpack
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xf ${OPENZEN_ARCHIVE_FILE}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  endif()

  find_package(OpenZen REQUIRED PATHS ${OPENZEN_BASE_FOLDER})
  SET(OPENZEN_TARGET_NAME OpenZen::OpenZen)
endif()

## Declare a catkin package
catkin_package()

## Build 
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(openzen_sensor_node
  src/OpenZenNode.cpp
)

target_link_libraries(openzen_sensor_node
  ${catkin_LIBRARIES}
  ${OPENZEN_TARGET_NAME}
)

add_dependencies(openzen_sensor_node ${catkin_EXPORTED_TARGETS})

## install node binary
install(TARGETS openzen_sensor_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(launch/openzen_test.launch)
endif()
