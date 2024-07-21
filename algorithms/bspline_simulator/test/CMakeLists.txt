cmake_minimum_required(VERSION 3.0)
project(VioSim)
set(CMAKE_CXX_STANDARD 17)

include_directories(../src)

file(GLOB SRCS "../src/*.cpp")

find_package(GTest REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

add_executable(test_vio_simulator test_vio_simulator.cpp ${SRCS})
target_link_libraries(test_vio_simulator gtest gtest_main pthread fmt::fmt ${Sophus_LIBRARIES})

