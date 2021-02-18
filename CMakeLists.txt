cmake_minimum_required(VERSION 2.8)
 
project(modern_robot)

find_package (Eigen3 REQUIRED)

include_directories ( "${EIGEN3_INCLUDE_DIR}" )
 
add_library(modern_robot SHARED src/modern_robotics.cpp src/modern_robotics.h)

set(TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/test_ikspace.cpp)

add_executable(robot_test ${TEST_SOURCES})

target_link_libraries(robot_test modern_robot)


