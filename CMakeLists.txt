cmake_minimum_required(VERSION 3.0.2)
project(nav_master)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  artificial_potential_fields
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  cv_bridge
  sensor_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

find_package(OpenCV REQUIRED)
################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
    FILES
    endPoint.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
    cfg/mpcParams.cfg
#   fg/DynReconf2.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pathSampler navigationStack local_costmap set_end_point
  CATKIN_DEPENDS artificial_potential_fields costmap_2d geometry_msgs pluginlib roscpp roscpp std_msgs tf2_ros 
  tf2_geometry_msgs dynamic_reconfigure message_runtime sensor_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(main src/main.cpp)
add_executable(goal_send src/goal_send.cpp)
add_executable(local_costmap src/local_costmap.cpp)
add_executable(set_end_point src/set_end_point.cpp)

target_link_libraries(main 
  ${catkin_LIBRARIES}
  navigationStack
  pathSampler
)

target_link_libraries(goal_send 
  ${catkin_LIBRARIES}
)

target_link_libraries(set_end_point 
  ${catkin_LIBRARIES}
)

add_library(pathSampler
  src/pathSampler.cpp
)

add_library(navigationStack
  src/navigationStack.cpp)


target_link_libraries(pathSampler
  ${catkin_LIBRARIES}
)

target_link_libraries(local_costmap 
  ${catkin_LIBRARIES}
)

target_link_libraries(navigationStack 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)



