cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(octree_pc_reader)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (octree_pc_reader octree_pc_reader.cpp)
target_link_libraries(octree_pc_reader ${PCL_LIBRARIES})
