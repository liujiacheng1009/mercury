cmake_minimum_required(VERSION 3.0)
project(nfr)
set(CMAKE_CXX_STANDARD 17)

include_directories(../src)
include_directories(../../bspline_simulator_basalt/src)
include_directories(../../imu_preintegration/src)

file(GLOB SRCS "../src/*.cpp")

find_package(GTest REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Sophus_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

add_executable(test_nfr test_nfr.cpp ${SRCS})
target_link_libraries(test_nfr gtest gtest_main pthread fmt::fmt ${Sophus_LIBRARIES} ${CERES_LIBRARIES})