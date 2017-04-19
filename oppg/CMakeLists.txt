cmake_minimum_required(VERSION 2.8.3)
project(mast)
add_definitions(-std=c++11)
enable_testing()

#include_directories (${PROJECT_SOURCE_DIR}/include/munkres/adapters)

set (SOURCE_FILES ./include/visilibity.cpp ./include/clipper.cpp SVG_builder.cpp tug_environment.cpp waypoint.cpp tug_shortest_path.cpp tug_point.cpp tug_boat.cpp tug_a_star_search.cpp tug_all_pairs_shortest_path.cpp tug_scheduler.cpp tug_assign_paths.cpp ./include/hungarian.cpp)

set (GTEST_DIR /Users/rebeccacox/GitHub/googletest/googletest)
set (MUNKRES_FILES ./include/munkres/munkres.cpp) # ./include/munkres/adapters/adapter.cpp)
   # ./include/munkres/adapters/boostmatrixadapter.cpp ./include/munkres/adapters/std2darrayadapter.cpp .#/include/munkres/adapters/std2dvectordapter.cpp)

# Mark cpp header files for installation
# install(
#   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
# )

add_subdirectory(${GTEST_DIR} googletest)
#add_subdirectory(${MUNKRES_DIR} munkres)


add_executable(tests tests.cpp ${SOURCE_FILES})
target_link_libraries(tests gtest gtest_main)
target_link_libraries(tests ${CMAKE_THREAD_LIBS_INIT}) #pthread?
target_link_libraries(tests ${GTEST_LIBRARIES})
add_test(NAME tests COMMAND tests)

#add_executable(main main.cpp ${SOURCE_FILES} ${MUNKRES_FILES})
#target_link_libraries(main)

#add_executable(demo demo1603.cpp ${SOURCE_FILES})
#target_link_libraries(demo)

#add_executable(simu tug_multiple_shortest_path_simulation.cpp ${SOURCE_FILES})
#target_link_libraries(simu)