cmake_minimum_required(VERSION 2.8.3)
project(geometry)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  external
)

catkin_package(
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tug_point.cpp src/tug_boat.cpp src/tug_environment.cpp src/SVG_builder.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS geometry)
  catkin_add_gtest(uTestGeometry test/test.cpp)
  target_link_libraries(uTestGeometry ${catkin_LIBRARIES})
endif()


install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)