cmake_minimum_required(VERSION 2.8.3)
project(door_finder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl pcl_ros roscpp sensor_msgs)


#get open cv
#include(FindPkgConfig)
#pkg_search_module(OPENCV REQUIRED opencv>=2.3 opencv-2.3.1)
#include_directories(${OPENCV_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES door_finder
  CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(my_pcl_tutorial
#   src/${PROJECT_NAME}/my_pcl_tutorial.cpp
# )


#set(MYHDRS plane_segmenter.h  strutils.h SimpleConfig.h edge_detector.h)
set(MYSRCS src/plane_segmenter.cpp src/edge_detector.cpp )
set(MYHDRS door_finder/SimpleConfig.h door_finder/strutils.h
     door_finder/edge_detector.h door_finder/plane_segmenter.h )

add_library( plane_segmenter ${MYSRCS} )
#add_executable (edge_detector src/door_finder.cpp)
add_executable(doorfindernode src/ros_interface.cpp)

set( LIBS plane_segmenter ${PCL_LIBRARIES} ${OPENCV_LDFLAGS} ${OpenCV_LIBS} )
#target_link_libraries (edge_detector ${LIBS} )
target_link_libraries (doorfindernode ${LIBS} ${catkin_LIBRARIES} )

## Declare a cpp executable


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(my_pcl_tutorial_node my_pcl_tutorial_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(my_pcl_tutorial_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_pcl_tutorial my_pcl_tutorial_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_pcl_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
