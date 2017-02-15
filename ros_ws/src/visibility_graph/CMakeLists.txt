cmake_minimum_required(VERSION 2.8.3)
project(visibility_graph)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

set (VISILIBITY_SOURCE_FILES src/visilibity.cpp src/clipper.cpp src/SVG_builder.cpp src/tug_environment.cpp)

catkin_package(
  INCLUDE_DIRS include/visibility_graph
)

include_directories( include/visibility_graph
  ${catkin_INCLUDE_DIRS}
)

# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
   PATTERN ".svn" EXCLUDE
 )

add_executable(visibility_graph_node src/visibility_graph_node.cpp ${VISILIBITY_SOURCE_FILES})
target_link_libraries(visibility_graph_node ${catkin_LIBRARIES})

