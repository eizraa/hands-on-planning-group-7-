cmake_minimum_required(VERSION 2.8.3)
project(swiftpro_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(swiftpro_rviz_node src/swiftpro_rviz_node.cpp)
add_dependencies(swiftpro_rviz_node swiftpro_generate_messages_cpp)
target_link_libraries(swiftpro_rviz_node ${catkin_LIBRARIES})