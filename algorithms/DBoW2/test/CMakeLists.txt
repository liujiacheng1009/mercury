cmake_minimum_required(VERSION 3.0)
project(DBoW2)
set(CMAKE_CXX_STANDARD 17)

include_directories(../include)

file(GLOB SRCS "../src/*.cpp")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(GTest REQUIRED)

add_executable(test_dbow2 test_dbow2.cpp ${SRCS})
target_link_libraries(test_dbow2 gtest gtest_main pthread ${OpenCV_LIBS})
file(COPY images DESTINATION ${CMAKE_BINARY_DIR}/)


