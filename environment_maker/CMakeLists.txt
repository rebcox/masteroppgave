cmake_minimum_required(VERSION 2.8.3)
project(environment_maker)
add_definitions(-std=c++11)
#enable_testing()

set (OPENCV_DIR ./opencv)
#set (SOURCE_FILES )

#set (GTEST_DIR ./googletest/googletest)

# Mark cpp header files for installation
# install(
#   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
# )

#add_subdirectory(${GTEST_DIR}

#add_subdirectory(${OPENCV_DIR})

#add_executable(tests tests.cpp ${SOURCE_FILES})
#target_link_libraries(tests gtest gtest_main)
#target_link_libraries(tests ${CMAKE_THREAD_LIBS_INIT}) #pthread?
#target_link_libraries(tests ${GTEST_LIBRARIES})
#add_test(NAME tests COMMAND tests)


find_package( OpenCV REQUIRED )
#target_link_libraries( DisplayImage ${OpenCV_LIBS} )

add_executable(main main.cpp) # ${SOURCE_FILES})
target_link_libraries(main ${OpenCV_LIBS} )