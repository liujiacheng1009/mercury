cmake_minimum_required(VERSION 3.10)

project(GpsConverter)

set(CMAKE_CXX_STANDARD 17)

include_directories(../include)

find_package(GTest REQUIRED)

add_executable(test_gps_converter test_gps_converter.cpp ../src/gps_converter.cpp)

target_link_libraries(test_gps_converter gtest gtest_main pthread)