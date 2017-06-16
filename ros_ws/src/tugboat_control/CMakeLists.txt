cmake_minimum_required(VERSION 2.8.3)
project(tugboat_control)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
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
    AvoidShipCollision.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)