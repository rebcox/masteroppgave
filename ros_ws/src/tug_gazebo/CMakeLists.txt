cmake_minimum_required(VERSION 2.8.3)
project(tug_gazebo)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  master
  external
  geometry
  std_msgs
  tugboat_control
)

catkin_package(
  CATKIN_DEPENDS tugboat_control
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(tug_startup_publisher src/tug_startup_publisher.cpp)
target_link_libraries(tug_startup_publisher ${catkin_LIBRARIES})

add_executable(tug_state_publisher src/tug_state_publisher.cpp)
target_link_libraries(tug_state_publisher ${catkin_LIBRARIES})

add_executable(goal_point_generator src/goal_point_generator.cpp)
target_link_libraries(goal_point_generator ${catkin_LIBRARIES})

add_executable(tug_location_publisher src/tug_location_publisher.cpp)
target_link_libraries(tug_location_publisher ${catkin_LIBRARIES})

add_executable(waypTugs_publisher src/waypTugs_publisher.cpp)
target_link_libraries(waypTugs_publisher ${catkin_LIBRARIES})