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

#add_message_files(
#  DIRECTORY
#  msg
#  FILES 
#  #Waypoint.msg
#  #BoatPose.msg
#  #ClearWaypoint.msg
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  tugboat_control
)
catkin_package(
  CATKIN_DEPENDS message_runtime tugboat_control
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_executable(master_node src/master_node.cpp src/tug_communicator.cpp)

add_dependencies(master_node external geometry search tugboat_control_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(master_node ${catkin_LIBRARIES})

