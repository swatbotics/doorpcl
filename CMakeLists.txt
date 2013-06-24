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
add_executable (edge_detector edge_detector.cpp)

target_link_libraries (edge_detector ${PCL_LIBRARIES})
target_link_libraries (edge_detector ${OPENCV_LDFLAGS})
