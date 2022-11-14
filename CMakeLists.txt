cmake_minimum_required(VERSION 3.23)
project(trace_plan_cpp)

set(CMAKE_CXX_STANDARD 14)

include_directories( ${PROJECT_SOURCE_DIR}/include)

add_executable(trace_plan_cpp
        include/jk5_stick.h
        src/jk5_stick.cpp
        main.cpp)

#find_package(Eigen3 REQUIRED)
#include_directories(/usr/include/eigen3/Eigen)
#find_package(orocos_kdl REQUIRED)
target_link_libraries(${PROJECT_NAME} orocos-kdl)