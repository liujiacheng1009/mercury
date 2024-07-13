cmake_minimum_required(VERSION 3.10)

project(CameraModel)

set(CMAKE_CXX_STANDARD 17)

include_directories(../include)

find_package(GTest REQUIRED)

add_executable(test_generic_camera test_generic_camera.cpp)

target_link_libraries(test_generic_camera gtest gtest_main pthread)