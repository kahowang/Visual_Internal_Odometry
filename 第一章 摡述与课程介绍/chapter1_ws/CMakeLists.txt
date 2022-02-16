#CMakeLists.txt
cmake_minimum_required(VERSION 3.0)
project(update_Quaterniond_SO3)

# 为使用 sophus，需要使用find_package命令找到它
find_package(Sophus REQUIRED)

# Eigen
include_directories("/usr/include/eigen3")
add_executable(update_Quaterniond_SO3 src/update_Quaterniond_SO3.cpp)
target_link_libraries(update_Quaterniond_SO3 Sophus::Sophus)

