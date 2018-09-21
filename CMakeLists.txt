cmake_minimum_required(VERSION 2.8.3)
project(novatel_gps)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  GpsXYZ.msg
  gps_position.msg
  gps_velocity.msg
  LogAll.msg
  PosType.msg
  Range.msg
  SatXYZInformation.msg
  SolStat.msg
  TrackStat.msg
  GpsTimeStat.msg
  MsgHeader.msg
  RangeInformation.msg
  ReceiverStat.msg
  SatXYZ.msg
  TrackStatChannel.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include(ExternalProject)
ExternalProject_Add(serialcom
    GIT_REPOSITORY https://github.com/Gastd/serialcom
    GIT_TAG master
    # PREFIX ${CMAKE_CURRENT_BINARY_DIR}/serialcom
    SOURCE_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src"
    BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/build"
    # CONFIGURE_COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" "${CMAKE_BINARY_DIR}/serialcom-src/"
    BUILD_COMMAND "${CMAKE_COMMAND}" --build .
    INSTALL_COMMAND ""
    TEST_COMMAND ""
)
# ExternalProject_Get_Property(serialcom install_dir)
ExternalProject_Get_Property(serialcom binary_dir)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/serialcom-src/include)
# message(${install_dir})

add_definitions(-std=c++11 -g)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES novatel_gps
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gps_node src/gps_node.cpp src/novatel_gps.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(gps_node serialcom ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gps_node
  ${catkin_LIBRARIES}
  ${binary_dir}/${CMAKE_FIND_LIBRARY_PREFIXES}serialcomlib.so
  -pthread
)
