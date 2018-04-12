cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(adaptive_voxel_size)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (adaptive_voxel_size adaptive_voxel_size.cpp)
target_link_libraries(adaptive_voxel_size ${PCL_LIBRARIES})
