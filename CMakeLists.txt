cmake_minimum_required(VERSION 2.8.3)
project(novatel_gps)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES novatel_gps
 CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
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
add_executable(gps_node src/gps_node.cpp src/gps.cpp src/serialcom.cpp)
add_executable(new_node src/new_node.cpp src/novatel_gps.cpp src/serialcom.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(gps_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(new_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gps_node
  ${catkin_LIBRARIES}
)
target_link_libraries(new_node
  ${catkin_LIBRARIES}
)
