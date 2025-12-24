cmake_minimum_required(VERSION 3.0)
project(kf_compare_test)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(kf_compare_test kf_compare_test.cpp)
target_link_libraries(kf_compare_test Eigen3::Eigen)
