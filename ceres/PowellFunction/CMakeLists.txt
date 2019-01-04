cmake_minimum_required(VERSION 2.8)
project(powellfunction)

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS})


add_executable(powellfunction PowellFunction.cpp)

target_link_libraries(powellfunction ${CERES_LIBRARIES})