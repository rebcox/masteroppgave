cmake_minimum_required(VERSION 2.8.3)
project(search)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry
  external
)
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
)

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/tug_a_star_search.cpp src/tug_all_pairs_shortest_path.cpp src/tug_shortest_path.cpp src/tug_route_around_ship.cpp)
add_dependencies(${PROJECT_NAME} external geometry )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS geometry search)
  catkin_add_gtest(uTestSearch test/test.cpp)
  target_link_libraries(uTestSearch ${catkin_LIBRARIES})
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