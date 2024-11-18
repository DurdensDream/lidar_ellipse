cmake_minimum_required(VERSION 3.10)
project(lidar_clustering)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_clustering main.cpp)
target_link_libraries(lidar_clustering ${PCL_LIBRARIES})