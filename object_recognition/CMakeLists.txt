cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

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

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_recognition
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    message_runtime
#  DEPENDS system_lib
)

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
add_dependencies(findingCenter object_recognition_gencpp)

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
