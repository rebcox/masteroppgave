cmake_minimum_required(VERSION 2.8.3)
project(simulation)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  external
  geometry
  visualization_msgs
  geometry_msgs
  urdf
  tf
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_executable(obstacles src/obstacles.cpp)
target_link_libraries(obstacles ${catkin_LIBRARIES})

add_executable(tug_broadcaster src/tug_broadcaster.cpp)
target_link_libraries(tug_broadcaster ${catkin_LIBRARIES})

add_executable(world_frame_broadcaster src/world_frame_broadcaster.cpp)
target_link_libraries(world_frame_broadcaster ${catkin_LIBRARIES})

add_executable(tug_mover src/tug_mover.cpp)
target_link_libraries(tug_mover ${catkin_LIBRARIES})

add_executable(tug_pose src/tug_pose.cpp)
target_link_libraries(tug_pose ${catkin_LIBRARIES})

#add_executable(tf_broadcaster src/tf_broadcaster.cpp)
#add_executable(tug src/tug.cpp)
#target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
#target_link_libraries(tug ${catkin_LIBRARIES})


#add_executable(tf_listener src/tf_listener.cpp)
#target_link_libraries(tf_listener ${catkin_LIBRARIES})