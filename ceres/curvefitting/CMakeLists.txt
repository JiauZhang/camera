cmake_minimum_required(VERSION 2.8)

project(curvefitting)

set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
#install(TARGETS curvefitting RUNTIME DESTINATION bin)
#/usr/include/c++/4.8/bits/c++0x_warning.h:32:2: error: 
#error This file requires compiler and library support for 
#the ISO C++ 2011 standard. This support is currently experimental, 
#and must be enabled with the -std=c++11 or -std=gnu++11 compiler options.

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(curvefitting curvefitting.cpp)
target_link_libraries(curvefitting ${CERES_LIBRARIES})


