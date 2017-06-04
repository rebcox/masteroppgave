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
)

add_message_files(
  DIRECTORY
    msg
  FILES 
    Waypoint.msg
    Waypoints.msg
    BoatPose.msg
    ClearWaypoint.msg
    Path.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    WaypointAvailable.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(master_node src/master_node.cpp src/tug_communicator.cpp)
add_dependencies(master_node external geometry search)
target_link_libraries(master_node ${catkin_LIBRARIES})

add_executable(tug_node src/tug_waypoint_publisher.cpp)
add_dependencies(tug_node)
target_link_libraries(tug_node ${catkin_LIBRARIES})

add_executable(current_waypoints_service src/current_waypoints_service.cpp)
add_dependencies(current_waypoints_service)
target_link_libraries(current_waypoints_service ${catkin_LIBRARIES})


add_executable(test_speed test/test_speed.cpp)
add_dependencies(test_speed external geometry search)
target_link_libraries(test_speed ${catkin_LIBRARIES})

add_executable(munkres_speed_test test/munkres_speed_test.cpp)
add_dependencies(munkres_speed_test external geometry search coordination)
target_link_libraries(munkres_speed_test ${catkin_LIBRARIES})