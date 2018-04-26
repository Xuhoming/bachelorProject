cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_preprocessing)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_preprocessing pcl_preprocessing.cpp)
target_link_libraries(pcl_preprocessing ${PCL_LIBRARIES})
