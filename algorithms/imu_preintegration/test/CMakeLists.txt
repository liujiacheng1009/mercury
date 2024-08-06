cmake_minimum_required(VERSION 3.0)
project(Bspline)
set(CMAKE_CXX_STANDARD 17)

include_directories(../src)
include_directories(../../bspline_simulator_basalt/src)

file(GLOB SRCS "../src/*.cpp")

find_package(GTest REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(test_preintegration test_preintegration.cpp ${SRCS})
target_link_libraries(test_preintegration gtest gtest_main pthread fmt::fmt ${Sophus_LIBRARIES} ${CERES_LIBRARIES})