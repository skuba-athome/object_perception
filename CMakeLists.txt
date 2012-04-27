cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

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
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
find_package(Flann)
include_directories(${FLANN_INCLUDE_DIRS})
include_directories("/usr/include/vtk-5.6")
include_directories(${HDF5_INCLUDE_DIR})
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_library(surf_libs	src/integral.cpp
#				src/utils.cpp
#				src/fasthessian.cpp
#				src/ipoint.cpp
#				src/surf.cpp)
#rosbuild_add_compile_flags(surf_libs "-fPIC")
#target_link_libraries(main surf_libs)
rosbuild_add_executable(main src/main.cpp)
target_link_libraries(main /usr/lib/libboost_system.so.1.46.1)
target_link_libraries(main /usr/lib/libboost_filesystem.so.1.46.1)
target_link_libraries(main /usr/lib/libhdf5.so.6)
#rosbuild_add_executable(capture src/capture.cpp)

rosbuild_add_executable(nearest_neighbors src/nearest_neighbors.cpp)
target_link_libraries(nearest_neighbors /usr/lib/libvtkCommon.so.5.6)
target_link_libraries(nearest_neighbors /usr/lib/libvtkRendering.so.5.6)
target_link_libraries(nearest_neighbors /usr/lib/libvtkFiltering.so.5.6)
target_link_libraries(nearest_neighbors /usr/lib/libboost_filesystem.so.1.46.1)
target_link_libraries(nearest_neighbors /usr/lib/libboost_system.so.1.46.1)
target_link_libraries(nearest_neighbors /usr/lib/libhdf5.so.6)


rosbuild_add_executable(ece src/ece.cpp)
rosbuild_add_executable(pcd_viewer src/pcd_viewer.cpp)
target_link_libraries(pcd_viewer /usr/lib/libvtkCommon.so.5.6)
target_link_libraries(pcd_viewer /usr/lib/libvtkRendering.so.5.6)
target_link_libraries(pcd_viewer /usr/lib/libvtkFiltering.so.5.6)


rosbuild_add_executable(tester src/test.cpp)
target_link_libraries(tester /usr/lib/libboost_filesystem.so.1.46.1)
target_link_libraries(tester /usr/lib/libhdf5.so.6)
target_link_libraries(tester /usr/lib/libboost_system.so.1.46.1)

rosbuild_add_executable(open_door src/open_door.cpp)
target_link_libraries(open_door /opt/ros/fuerte/lib/libopencv_core.so)

