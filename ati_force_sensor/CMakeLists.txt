cmake_minimum_required(VERSION 2.8.3)
project(ati_force_sensor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	tf
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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
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
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ati_force_sensor
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
link_directories(/opt/ros/noetic/lib)
link_directories(/home/jiyguo/ur_ws/devel/lib)
link_directories(/home/jiyguo/ur_ws/devel/lib/ati_force_sensor)
link_libraries(libxmlrpcpp.so)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
	src/MCC_ATI
	src/ati_force_sensor/include/robotController
	/usr/local/include/kdl
	/usr/include/python2.7
	/usr/include/eigen3
	/opt/ros/noetic/include
		../uu_robot/include
		../robot_kin/include
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ati_force_sensor.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ati_force_sensor_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
add_library(ati_library src/myATI.cpp
        src/MCC_ATI/ftconfig.c
        src/MCC_ATI/ftrt.c
        src/MCC_ATI/dom.c
        src/MCC_ATI/expatls.c
        src/MCC_ATI/node.c
        src/MCC_ATI/stack.c
        src/MCC_ATI/xmlparse.c
        src/MCC_ATI/xmlrole.c
        src/MCC_ATI/xmltok.c)
target_link_libraries(ati_library orocos-kdl mccusb hidapi-libusb m usb-1.0)

#add_executable(ATI_pub_test src/test_ATI.cpp)
#target_link_libraries(ATI_pub_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library robot_kin_)

#add_executable(calAverageForces src/calAverageForces.cpp)
#target_link_libraries(calAverageForces ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library)
#
#add_executable(testGravityCompensation src/testGravityCompensation.cpp)
#target_link_libraries(testGravityCompensation ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library dual_arm_robot)
#
#add_executable(rigidbodyTranslation src/rigidbody_translation.cpp)
#target_link_libraries(rigidbodyTranslation ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library orocos-kdl)
#
#add_executable(stiffnesscontrol src/stiffnesscontrol.cpp)
#target_link_libraries(stiffnesscontrol ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library dual_arm_robot)
#
#add_executable(auto_calibre_gravity_init_down src/auto_calibre_gravity_init_down.cpp)
#target_link_libraries(auto_calibre_gravity_init_down ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library dual_arm_robot)
#

#add_executable(auto_calibre_gravity src/robotController/robotController.cpp src/auto_calibre_gravity.cpp)
#target_link_libraries(auto_calibre_gravity ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library  robot_kin_)
#
#add_executable(test_auto_gravity_compensation src/robotController/robotController.cpp  src/test_auto_gravity_compensation.cpp)
#target_link_libraries(test_auto_gravity_compensation ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library  robot_kin_)
#
#add_executable(test_auto_gravity_compensation_down_init src/test_auto_gravity_compensation_down_init.cpp)
#target_link_libraries(test_auto_gravity_compensation_down_init ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library dual_arm_robot)
#
#add_executable(test_read_bias src/test_readbias.cpp)
#target_link_libraries(test_read_bias ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library dual_arm_robot)
#
#add_executable(compensated_force src/compensated_force.cpp)
#target_link_libraries(compensated_force ${catkin_LIBRARIES} ${Boost_LIBRARIES} ati_library dual_arm_robot)
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ati_force_sensor.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
