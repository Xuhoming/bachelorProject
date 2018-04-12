cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_octree_grid)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_octree_grid pcl_octree_grid.cpp)
target_link_libraries(pcl_octree_grid ${PCL_LIBRARIES})
