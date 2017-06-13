cmake_minimum_required(VERSION 2.8.3)
project(master)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  external
  geometry
  search
  coordination
  message_generation
  std_msgs
  tugboat_control
)

catkin_package(
  CATKIN_DEPENDS message_runtime tugboat_control
  INCLUDE_DIRS include
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(master_node src/master_node.cpp src/tug_communicator.cpp)
add_dependencies(master_node external geometry search)
target_link_libraries(master_node ${catkin_LIBRARIES})

add_executable(avoid_ship_collision src/avoid_ship_collision_service.cpp)
add_dependencies(avoid_ship_collision geometry search)
target_link_libraries(avoid_ship_collision ${catkin_LIBRARIES})

add_executable(tug_node src/main_tug.cpp src/tug_waypoint_publisher.cpp)
target_link_libraries(tug_node ${catkin_LIBRARIES})

add_executable(current_waypoints_service src/current_waypoints_service.cpp)
target_link_libraries(current_waypoints_service ${catkin_LIBRARIES})


add_executable(test_speed test/test_speed.cpp)
target_link_libraries(test_speed ${catkin_LIBRARIES})