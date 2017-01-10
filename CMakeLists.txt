cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer visualization/cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

add_executable (kinect_logger visualization/kinect_logger.cpp)
target_link_libraries (kinect_logger ${PCL_LIBRARIES})
