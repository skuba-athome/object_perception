cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
  roscpp
  image_transport
  message_generation
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(pcl_ros REQUIRED)
include_directories(${pcl_ros_INCLUDE_DIRS})


find_package(tf REQUIRED)
include_directories(${tf_INCLUDE_DIRS})

#Reminder: these are inclued in faute, may need to be add here as well.
#include_directories("/usr/include/vtk-5.8")
#include_directories("/usr/include/openmpi")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
  Object.msg
  ObjectContainer.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  classifyObject.srv
  verifyObject.srv
  findObject.srv
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
   std_msgs  # Or other packages containing msgs
   geometry_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES object_recognition
  CATKIN_DEPENDS roscpp rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(object_recognition
#   src/${PROJECT_NAME}/object_recognition.cpp
# )

## Declare a cpp executable
# add_executable(object_recognition_node src/object_recognition_node.cpp)


add_executable(image_view src/image_view.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES})
target_link_libraries(image_view ${OpenCV_LIBRARIES})



add_executable(click_object src/click_object.cpp)
target_link_libraries(click_object ${catkin_LIBRARIES})
target_link_libraries(click_object ${OpenCV_LIBRARIES})
target_link_libraries(click_object ${PCL_LIBRARIES})
target_link_libraries(click_object ${tf_LIBRARIES})
target_link_libraries(click_object ${pcl_ros_LIBRARIES})



add_executable(findingCenter src/findingCenter.cpp)
target_link_libraries(findingCenter ${catkin_LIBRARIES})
target_link_libraries(findingCenter ${OpenCV_LIBRARIES})
target_link_libraries(findingCenter ${PCL_LIBRARIES})
target_link_libraries(findingCenter ${tf_LIBRARIES})
target_link_libraries(findingCenter ${pcl_ros_LIBRARIES})
add_dependencies(findingCenter manipulator)


add_executable(drawKeypoints src/drawKeypoints.cpp)
target_link_libraries(drawKeypoints ${catkin_LIBRARIES})
target_link_libraries(drawKeypoints ${OpenCV_LIBRARIES})

add_executable(drawKeypointsAll src/drawKeypointsAll.cpp)
target_link_libraries(drawKeypointsAll ${catkin_LIBRARIES})
target_link_libraries(drawKeypointsAll ${OpenCV_LIBRARIES})

add_executable(drawKeypointsArch src/drawKeypointsArch.cpp)
target_link_libraries(drawKeypointsArch ${catkin_LIBRARIES})
target_link_libraries(drawKeypointsArch ${OpenCV_LIBRARIES})

add_executable(extractSURF src/extractSURF.cpp)
target_link_libraries(extractSURF ${catkin_LIBRARIES})
target_link_libraries(extractSURF ${OpenCV_LIBRARIES})

add_executable(verify_object src/verify_object.cpp)
target_link_libraries(verify_object ${catkin_LIBRARIES})
target_link_libraries(verify_object ${OpenCV_LIBRARIES})

add_executable(tune_cropped src/tune_cropped.cpp)
target_link_libraries(tune_cropped ${catkin_LIBRARIES})
target_link_libraries(tune_cropped ${OpenCV_LIBRARIES})
target_link_libraries(tune_cropped ${PCL_LIBRARIES})
target_link_libraries(tune_cropped ${tf_LIBRARIES})
target_link_libraries(tune_cropped ${pcl_ros_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(object_recognition_node object_recognition_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(object_recognition_node
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
# install(TARGETS object_recognition object_recognition_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_recognition.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
