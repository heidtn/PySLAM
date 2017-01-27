cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cloud_viewer)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#add_executable (cloud_viewer visualization/cloud_viewer.cpp)
#target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

#add_executable (kinect_logger visualization/kinect_logger.cpp)
#target_link_libraries (kinect_logger ${PCL_LIBRARIES})

#add_executable (icp_viewer visualization/icp_viewer.cpp)
#target_link_libraries( icp_viewer ${PCL_LIBRARIES})

#add_executable (correspondence_viewer visualization/correspondence_viewer.cpp)
#target_link_libraries( correspondence_viewer ${PCL_LIBRARIES})

add_executable (akaze_orb visualization/akaze_orb.cpp)
target_link_libraries( akaze_orb ${OpenCV_LIBS} )
