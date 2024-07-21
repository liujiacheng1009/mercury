cmake_minimum_required(VERSION 3.0)
project(VioSim)
set(CMAKE_CXX_STANDARD 17)

include_directories(../src)

file(GLOB SRCS "../src/*.cpp")

find_package(GTest REQUIRED)

add_executable(test_logger test_logger.cpp ${SRCS})
target_link_libraries(test_logger gtest gtest_main pthread)

