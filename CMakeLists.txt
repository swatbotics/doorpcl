cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set (CMAKE_CXX_FLAGS "-g ")

project(edge_detector)

find_package(PCL 1.2 REQUIRED)
include(FindPkgConfig)
pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(HDRS plane_segmenter.h )
set(SRCS plane_segmenter.cpp )

add_library( plane_segmenter ${HDRS} ${SRCS} )
add_executable (edge_detector ${HDRS} edge_detector.cpp)

set( LIBS plane_segmenter ${PCL_LIBRARIES} ${OPENCV_LDFLAGS} )
target_link_libraries (edge_detector ${LIBS} )
