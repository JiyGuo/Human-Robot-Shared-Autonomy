cmake_minimum_required(VERSION 2.8.3)
project(gmm)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
cmake_modules
roscpp
rospy
std_msgs
message_generation
)
find_package(Eigen REQUIRED)


#set(tinyxml2_DIR /usr/lib/x86_64-linux-gnu)
#set_target_properties( libtinyxml2
#        PROPERTIES IMPORTED_LOCATION
#        ${tinyxml2_DIR}/libtinyxml2.so)

link_directories(/usr/lib/x86_64-linux-gnu)
link_directories(../../devel/lib)

# add_message_files(
#    FILES
#  )

add_service_files(
   FILES
   GetConstraint.srv
 )

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS common_msgs image_common message_runtime
)

include_directories(
  include
  ../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# add_library(gmm  src/gmm.cpp )
# target_link_libraries(gmm   ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(gmm_test nodes/gmm_test.cpp src/gmm.cpp)
target_link_libraries(gmm_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES} tinyxml2)

# add_executable(dmp_learn nodes/dmp_learn.cpp)
# target_link_libraries(dmp_learn ${catkin_LIBRARIES} ${Eigen_LIBRARIES} )

# add_executable(dmp_gen nodes/dmp_gen.cpp)
# target_link_libraries(dmp_gen ${catkin_LIBRARIES} ${Eigen_LIBRARIES} )

# add_executable(classify_test nodes/classify_test.cpp src/confidance.cpp)
# target_link_libraries(classify_test ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
