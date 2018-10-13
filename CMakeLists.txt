cmake_minimum_required(VERSION 2.8.3)
project(joy_cont)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES joy_cont
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/joy_cont.cpp
 )


add_executable(${PROJECT_NAME}_node src/joy_cont.cpp)


set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME joy_cont)

 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

