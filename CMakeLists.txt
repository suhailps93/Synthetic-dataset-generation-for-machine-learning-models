cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable (record src/record.cpp)
target_link_libraries (record ${PCL_LIBRARIES})


