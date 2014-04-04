cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#add-------------
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
#----------------

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
#find_package(Flann)
#include_directories(${HDF5_INCLUDE_DIR})
#include_directories(${FLANN_INCLUDE_DIRS})
#include_directories(${FLANN_INCLUDE_DIRS})
include_directories("/usr/include/vtk-5.8")
include_directories("/usr/include/openmpi")
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
#
#rosbuild_add_library(surf_libs	src/integral.cpp
#				src/utils.cpp
#				src/fasthessian.cpp
#				src/ipoint.cpp
#				src/surf.cpp)
#rosbuild_add_compile_flags(surf_libs "-fPIC")
#
#rosbuild_add_executable(tester src/test.cpp)
#target_link_libraries(tester /usr/lib/libboost_filesystem.so.1.46.1)
#target_link_libraries(tester /usr/lib/libhdf5.so.6)
#target_link_libraries(tester /usr/lib/libboost_system.so.1.46.1)
#
#rosbuild_add_executable(tester_color src/test_color.cpp)
#target_link_libraries(tester_color /usr/lib/libboost_filesystem.so.1.46.1)
#target_link_libraries(tester_color /usr/lib/libhdf5.so.6)
#target_link_libraries(tester_color /usr/lib/libboost_system.so.1.46.1)
#
#
#rosbuild_add_executable(object_surf src/mainSurf.cpp)
#target_link_libraries(object_surf surf_libs)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_core.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_imgproc.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_highgui.so)
#target_link_libraries(surf_libs   /opt/ros/fuerte/lib/libopencv_objdetect.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_contrib.so)
#target_link_libraries(surf_libs /opt/ros/fuerte/lib/libopencv_calib3d.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_legacy.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_flann.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_stitching.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_ts.so)
#target_link_libraries(object_surf /opt/ros/fuerte/lib/libopencv_video.so)
#
#rosbuild_add_executable(main src/main.cpp)
#target_link_libraries(main /usr/lib/libboost_system.so.1.46.1)
#target_link_libraries(main /usr/lib/libboost_filesystem.so.1.46.1)
#target_link_libraries(main /usr/lib/libhdf5.so.6)
##rosbuild_add_executable(capture src/capture.cpp)
#
#rosbuild_add_executable(nearest_neighbors src/nearest_neighbors.cpp)
#target_link_libraries(nearest_neighbors /usr/lib/libvtkCommon.so.5.8)
#target_link_libraries(nearest_neighbors /usr/lib/libvtkRendering.so.5.8)
#target_link_libraries(nearest_neighbors /usr/lib/libvtkFiltering.so.5.8)
#target_link_libraries(nearest_neighbors /usr/lib/libboost_filesystem.so.1.46.1)
#target_link_libraries(nearest_neighbors /usr/lib/libboost_system.so.1.46.1)
#target_link_libraries(nearest_neighbors /usr/lib/libhdf5.so.6)
#
#
#rosbuild_add_executable(ece src/ece.cpp)
#rosbuild_add_executable(pcd_viewer src/pcd_viewer.cpp)
#target_link_libraries(pcd_viewer /usr/lib/libvtkCommon.so.5.8)
#target_link_libraries(pcd_viewer /usr/lib/libvtkRendering.so.5.8)
#target_link_libraries(pcd_viewer /usr/lib/libvtkFiltering.so.5.8)
#
#rosbuild_add_executable(pcd_color_viewer src/pcd_color_viewer.cpp)
#target_link_libraries(pcd_color_viewer /usr/lib/libvtkCommon.so.5.8)
#target_link_libraries(pcd_color_viewer /usr/lib/libvtkRendering.so.5.8)
#target_link_libraries(pcd_color_viewer /usr/lib/libvtkFiltering.so.5.8)
#
#
#rosbuild_add_executable(open_door src/open_door.cpp)
#target_link_libraries(open_door /opt/ros/fuerte/lib/libopencv_core.so)
#
#rosbuild_add_executable(color src/obj_color.cpp)
#rosbuild_add_executable(color2 src/color.cpp)
#rosbuild_add_executable(pic src/pic.cpp)
#



rosbuild_add_executable(findingCenter src/findingCenter.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(findingCenter filesystem system)

rosbuild_add_executable(extractSURF src/extractSURF.cpp)
