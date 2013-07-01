cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_CXX_FLAGS "-g")

project(pcd_write)
project(openni_grabber)
project(random_sample_consensus)
project(planar_segmentation)
project(openni_viewer_simple)
project(edge_detector)

find_package(PCL 1.2 REQUIRED)
include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
<<<<<<< HEAD

set(HDRS plane_segmenter.h  strutils.h SimpleConfig.h )
set(SRCS plane_segmenter.cpp )

add_library( plane_segmenter ${HDRS} ${SRCS} )
add_executable (edge_detector ${HDRS} edge_detector.cpp)

set( LIBS plane_segmenter ${PCL_LIBRARIES} ${OPENCV_LDFLAGS} )
target_link_libraries (edge_detector ${LIBS} )
=======

add_executable (pcd_write pcd_write.cpp)
add_executable (openni_grabber openni_grabber.cpp)
add_executable (random_sample_consensus random_sample_consensus.cpp)
add_executable (planar_segmentation planar_segmentation.cpp)
add_executable (openni_viewer_simple openni_viewer_simple.cpp)
add_executable (edge_detector edge_detector.cpp)

target_link_libraries (pcd_write ${PCL_LIBRARIES})
target_link_libraries (openni_grabber ${PCL_LIBRARIES})
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})
target_link_libraries (openni_viewer_simple ${PCL_LIBRARIES})
target_link_libraries (edge_detector ${PCL_LIBRARIES})
target_link_libraries (edge_detector ${OPENCV_LDFLAGS})
>>>>>>> parent of 879ba09... Changed the cmakefile to exclude extraneous code
