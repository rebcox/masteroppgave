cmake_minimum_required(VERSION 2.8.3)
project(simulation)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  external
  geometry
  visualization_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_executable(obstacles src/obstacles.cpp)
target_link_libraries(obstacles ${catkin_LIBRARIES})