cmake_minimum_required(VERSION 2.8.3)
project(mast)
add_definitions(-std=c++11)
enable_testing()

set(COORDINATION_FILES ./coordination/src/tug_scheduler.cpp ./coordination/src/tug_assign_paths.cpp)

set(SEARCH_FILES ./search/src/tug_a_star_search.cpp ./search/src/tug_all_pairs_shortest_path.cpp ./search/src/tug_shortest_path.cpp)

set(GEOMETRY_FILES ./geometry/src/tug_point.cpp ./geometry/src/tug_waypoint.cpp ./geometry/src/tug_boat.cpp ./geometry/src/tug_environment.cpp)

set (EXTERNAL_FILES ./external/src/visilibity.cpp ./external/src/clipper.cpp ./external/src/munkres.cpp ./external/src/SVG_builder.cpp)

set(SOURCE_FILES ${EXTERNAL_FILES} ${GEOMETRY_FILES} ${SEARCH_FILES} ${COORDINATION_FILES})

set (GTEST_DIR ./googletest/googletest)

add_subdirectory(${GTEST_DIR} googletest)

add_executable(tests tests.cpp ${SOURCE_FILES})
target_link_libraries(tests gtest gtest_main)
target_link_libraries(tests ${CMAKE_THREAD_LIBS_INIT}) #pthread?
target_link_libraries(tests ${GTEST_LIBRARIES})
add_test(NAME tests COMMAND tests)

add_executable(main main.cpp ${SOURCE_FILES})
target_link_libraries(main)

#add_executable(demo demo1603.cpp ${SOURCE_FILES})
#target_link_libraries(demo)

#add_executable(simu tug_multiple_shortest_path_simulation.cpp ${SOURCE_FILES})
#target_link_libraries(simu)