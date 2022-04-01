cmake_minimum_required(VERSION 3.0.2)
project(my_controller)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    geometry_msgs
    message_generation
    roscpp
    rospy
    std_msgs
    cv_bridge
    image_transport
    pcl_ros
    sensor_msgs
    control_msgs
    move_base_msgs
    trajectory_msgs
    cmake_modules
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    moveit_visual_tools
)

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocrtoc_solution
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
#)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs actionlib  move_base_msgs  control_msgs trajectory_msgs interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib   actionlib_msgs
)

link_directories(/opt/ros/melodic/lib)
link_directories(/fmauch_universal_robot/ur_kinematics)
link_directories(/../../../opt/ros/melodic/lib)
LINK_DIRECTORIES(/usr/lib/x86_64-linux-gnu)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        /usr/include/eigen3
        ../robot_kin/include
        ../dmp/include
        ../../devel/include
        ../fmauch_universal_robot/ur_kinematics/include
)

install(DIRECTORY
  controller
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#add_executable(test1 src/test.cpp
#                   src/robotController/robotController.cpp)
#target_link_libraries(test1 ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl robot_kin)


add_executable(robotControllerTest src/robotController/robotControllerTest.cpp
                   src/robotController/robotController.cpp)
target_link_libraries(robotControllerTest ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl robot_kin)

#add_executable(dmp_robot src/dmp_step.cpp
#        src/robotController/robotController.cpp)
#target_link_libraries(dmp_robot ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl robot_kin tinyxml2 dmp)



#
#add_executable(loadCameraImage src/readImage/loadCameraImage.cpp
#        src/readImage/readImage.cpp)
#target_link_libraries(loadCameraImage ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl)
#
#add_executable(grasp_cubic src/grasp_cubic.cpp
#        src/readImage/readImage.cpp
#        src/robotController/robotController.cpp
#        )
#target_link_libraries(grasp_cubic ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} orocos-kdl robot_kin)
#
#add_executable(calibration2d src/readChess.cpp
#        src/readImage/readImage.cpp
#        src/robotController/robotController.cpp
#        )
#target_link_libraries(calibration2d ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} orocos-kdl robot_kin)
#
#add_executable(get_pos src/get_pos.cpp
#        src/robotController/robotController.cpp
#        )
#target_link_libraries(get_pos ${catkin_LIBRARIES} ${Boost_LIBRARIES} orocos-kdl robot_kin)
#
#add_executable(detect_test src/detect_test.cpp
#        src/readImage/readImage.cpp
#        )
#target_link_libraries(detect_test ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} orocos-kdl)


# --------------------------- moveIt -------------------------------------- #

#
#add_executable(simpleTest src/moveIt/simpleTest.cpp)
#target_link_libraries(simpleTest ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#
#add_executable(collisionTest src/moveIt/collisionTest.cpp)
#target_link_libraries(collisionTest ${catkin_LIBRARIES} ${Boost_LIBRARIES})