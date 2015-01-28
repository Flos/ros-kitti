cmake_minimum_required(VERSION 2.8.3)
project(kitti)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_geometry
  image_transport
  pcl_conversions
  pcl_ros
  cv_bridge
  nodelet
  dynamic_reconfigure
  roscpp
  rospy
  rostime
  sensor_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/kitti_export.cfg
)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES 
  Sync_msg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

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
  LIBRARIES kitti
  CATKIN_DEPENDS image_geometry image_transport pcl_conversions pcl_ros cv_bridge nodelet roscpp rospy rostime sensor_msgs std_msgs tf
  DEPENDS system_lib
)

###########
## Build ##
###########

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(BEFORE 	${PCL_INCLUDE_DIRS}  
							${OpenCV_INCLUDE_DIRS}
  							${Boost_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS}
				 ${OpenCV_LIBRARY_DIRS}
)

add_definitions( ${PCL_DEFINITIONS}
				 ${OpenCV_DEFINITIONS}
)

if (NOT PCL_FOUND)
    MESSAGE(FATAL_ERROR "PCL not found.\n")
endif (NOT PCL_FOUND)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

set(common
  src/${PROJECT_NAME}/common/camera_subscriber.cpp
  src/${PROJECT_NAME}/common/pointcloud_subscriber.cpp
  src/${PROJECT_NAME}/common/generic_sync_publisher.cpp
  src/${PROJECT_NAME}/common/serialization/camera_list.cpp
  src/${PROJECT_NAME}/common/serialization/camera.cpp
  src/${PROJECT_NAME}/common/serialization/dataset.cpp
  src/${PROJECT_NAME}/common/serialization/file_list.cpp
  src/${PROJECT_NAME}/common/serialization/filenames.cpp
  src/${PROJECT_NAME}/common/serialization/serializable.cpp
  src/${PROJECT_NAME}/common/serialization/tf.cpp
  src/${PROJECT_NAME}/common/generic_subscriber.cpp
)

## Declare a cpp library
add_library(kitti_export
  src/${PROJECT_NAME}/export/nodelet.cpp
)

add_dependencies(kitti_export
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
  kitti
)

target_link_libraries(kitti_export
  ${catkin_LIBRARIES}
  kitti
)

add_executable(export_node
  src/${PROJECT_NAME}/export/node.cpp
  ${common}
)

add_dependencies(export_node
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
  kitti
)

target_link_libraries(export_node
  ${catkin_LIBRARIES}
  kitti
)

## Create lib to share
add_library(kitti
  ${common}
)

add_dependencies(kitti
  ${PROJECT_NAME}_gencfg
  ${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(kitti
  ${catkin_LIBRARIES}
)



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
# install(TARGETS kitti kitti_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kitti.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
