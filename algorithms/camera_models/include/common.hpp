#pragma once
#include <vector>
#include <Eigen/Dense>

#define UNUSED(x) (void)(x)

template<typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;