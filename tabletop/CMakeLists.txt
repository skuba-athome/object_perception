cmake_minimum_required(VERSION 2.8.3)
project(tabletop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  shape_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
include_directories(include ${EIGEN_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_message_files(FILES
  Table.msg
  #TabletopDetectionResult.msg
)


add_service_files(FILES
  TabletopSegmentation.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  sensor_msgs
  shape_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs    
    message_runtime    
    pcl_ros
    pcl_conversions
    roscpp
    sensor_msgs
    shape_msgs
    std_msgs
    tf
    visualization_msgs
  DEPENDS eigen boost
  INCLUDE_DIRS include
  LIBRARIES
    marker_generator
)

add_library(marker_generator src/marker_generator.cpp )
target_link_libraries(marker_generator ${catkin_LIBRARIES})
add_dependencies(marker_generator ${PROJECT_NAME}_gencpp)

add_executable(tabletop_segmentation src/tabletop_segmentation.cpp )
target_link_libraries(tabletop_segmentation marker_generator ${catkin_LIBRARIES})

add_executable(ping_tabletop_segmentation src/ping_tabletop_segmentation.cpp)
target_link_libraries(ping_tabletop_segmentation ${catkin_LIBRARIES})
add_dependencies(ping_tabletop_segmentation ${PROJECT_NAME}_gencpp)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS
    marker_generator
    tabletop_segmentation
    ping_tabletop_segmentation
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

