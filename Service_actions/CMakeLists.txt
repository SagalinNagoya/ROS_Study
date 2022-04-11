cmake_minimum_required(VERSION 3.0.2)
project(mastering_ros_demo_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
#  message_generation
#  message_runtime
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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
##     * add a exec_depend tag for "message_runtime"demo_service_client
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET tomessage_generation

#add_message_files(
#  FILES
#  demo_msg.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  demo_srv.srv
#)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Demo_action.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
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
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also needdemo_service_server
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mastering_ros_demo_pkg
#  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mastering_ros_demo_pkg.cppdemo_service_serverTS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(demo_topic_publisher src/demo_topic_publisher.cpp)
#add_executable(demo_topic_subscriber src/demo_topic_subscriber.cpp)
#add_executable(demo_msg_publisher ./src/demo_msg_publisher.cpp)
#add_executable(demo_msg_subscriber ./src/demo_msg_subscriber.cpp)
#add_executable(demo_service_server ./src/demo_service_server.cpp)
#add_executable(demo_service_server ./src/demo_service_server.cpp)
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(demo_msg_publisher mastering_ros_demo_pkg_generate_messages_cpp)
#add_dependencies(demo_msg_subscriber mastering_ros_demo_pkg_generate_messages_cpp)
#add_dependencies(demo_service_server mastering_ros_demo_pkg_generate_messages_cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(demo_topic_publisher ${catkin_LIBRARIES})
#target_link_libraries(demo_topic_subscriber ${catkin_LIBRARIES})
#target_link_libraries(demo_msg_publisher ${catkin_LIBRARIES})
#target_link_libraries(demo_msg_subscriber ${catkin_LIBRARIES})
#target_link_libraries(demo_service_server ${catkin_LIBRARIES})
#target_link_libraries(demo_service_client ${catkin_LIBRARIES})

#Service server
#add_executable(demo_service_server ./src/demo_service_server.cpp)
#add_dependencies(demo_service_server mastering_ros_demo_pkg_generate_messages_cpp)
#target_link_libraries(demo_service_server ${catkin_LIBRARIES})

#Service client
#add_executable(demo_service_client ./src/demo_service_client.cpp)
#add_dependencies(demo_service_client mastering_ros_demo_pkg_generate_messages_cpp)
#target_link_libraries(demo_service_client ${catkin_LIBRARIES})



#Action server
add_executable(demo_action_server ./src/demo_action_server.cpp)
add_dependencies(demo_action_server mastering_ros_demo_pkg_generate_messages_cpp)
target_link_libraries(demo_action_server ${catkin_LIBRARIES})
#Action client
add_executable(demo_action_client ./src/demo_action_client.cpp)
add_dependencies(demo_action_client mastering_ros_demo_pkg_generate_messages_cpp)
target_link_libraries(demo_action_client ${catkin_LIBRARIES})

#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMSmastering_ros_demo_pkg
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}mastering_ros_demo_pkgag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mastering_ros_demo_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
