cmake_minimum_required(VERSION 2.8.3)
project(tug_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  master
  external
  geometry
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tug_state_publisher src/tug_state_publisher.cpp)
target_link_libraries(tug_state_publisher ${catkin_LIBRARIES})

add_executable(goal_point_generator src/goal_point_generator.cpp)
target_link_libraries(goal_point_generator ${catkin_LIBRARIES})

add_executable(tug_location_publisher src/tug_location_publisher.cpp)
target_link_libraries(tug_location_publisher ${catkin_LIBRARIES})