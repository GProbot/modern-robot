cmake_minimum_required(VERSION 2.8)
 
project(modern_robot)

find_package (Eigen3 REQUIRED)

include_directories ( "${EIGEN3_INCLUDE_DIR}" )

add_library(modern_robot SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/modern_robotics.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/modern_robotics.h)

add_executable(robot_test ${CMAKE_CURRENT_SOURCE_DIR}/src/test_ikspace.cpp)

target_link_libraries(robot_test modern_robot)
