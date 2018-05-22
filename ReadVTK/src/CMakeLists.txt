
cmake_minimum_required(VERSION 2.8)
 
PROJECT(ReadVTK)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(ReadVTK MACOSX_BUNDLE ReadVTK.cxx )
 
target_link_libraries(ReadVTK ${VTK_LIBRARIES})
