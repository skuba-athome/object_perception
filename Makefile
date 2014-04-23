# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/skuba/skuba_athome/object_perception

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/skuba/skuba_athome/object_perception

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/skuba/skuba_athome/object_perception/CMakeFiles /home/skuba/skuba_athome/object_perception/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/skuba/skuba_athome/object_perception/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_cpp

# Build rule for target.
ROSBUILD_genmsg_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_cpp
.PHONY : ROSBUILD_genmsg_cpp

# fast build rule for target.
ROSBUILD_genmsg_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_cpp.dir/build.make CMakeFiles/ROSBUILD_genmsg_cpp.dir/build
.PHONY : ROSBUILD_genmsg_cpp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_lisp

# Build rule for target.
ROSBUILD_genmsg_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_lisp
.PHONY : ROSBUILD_genmsg_lisp

# fast build rule for target.
ROSBUILD_genmsg_lisp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_lisp.dir/build.make CMakeFiles/ROSBUILD_genmsg_lisp.dir/build
.PHONY : ROSBUILD_genmsg_lisp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_py

# Build rule for target.
ROSBUILD_genmsg_py: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_py
.PHONY : ROSBUILD_genmsg_py

# fast build rule for target.
ROSBUILD_genmsg_py/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_py.dir/build.make CMakeFiles/ROSBUILD_genmsg_py.dir/build
.PHONY : ROSBUILD_genmsg_py/fast

#=============================================================================
# Target rules for targets named ROSBUILD_gensrv_cpp

# Build rule for target.
ROSBUILD_gensrv_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gensrv_cpp
.PHONY : ROSBUILD_gensrv_cpp

# fast build rule for target.
ROSBUILD_gensrv_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gensrv_cpp.dir/build.make CMakeFiles/ROSBUILD_gensrv_cpp.dir/build
.PHONY : ROSBUILD_gensrv_cpp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_gensrv_lisp

# Build rule for target.
ROSBUILD_gensrv_lisp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gensrv_lisp
.PHONY : ROSBUILD_gensrv_lisp

# fast build rule for target.
ROSBUILD_gensrv_lisp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gensrv_lisp.dir/build.make CMakeFiles/ROSBUILD_gensrv_lisp.dir/build
.PHONY : ROSBUILD_gensrv_lisp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_gensrv_py

# Build rule for target.
ROSBUILD_gensrv_py: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gensrv_py
.PHONY : ROSBUILD_gensrv_py

# fast build rule for target.
ROSBUILD_gensrv_py/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gensrv_py.dir/build.make CMakeFiles/ROSBUILD_gensrv_py.dir/build
.PHONY : ROSBUILD_gensrv_py/fast

#=============================================================================
# Target rules for targets named clean-test-results

# Build rule for target.
clean-test-results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 clean-test-results
.PHONY : clean-test-results

# fast build rule for target.
clean-test-results/fast:
	$(MAKE) -f CMakeFiles/clean-test-results.dir/build.make CMakeFiles/clean-test-results.dir/build
.PHONY : clean-test-results/fast

#=============================================================================
# Target rules for targets named drawKeypoints

# Build rule for target.
drawKeypoints: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 drawKeypoints
.PHONY : drawKeypoints

# fast build rule for target.
drawKeypoints/fast:
	$(MAKE) -f CMakeFiles/drawKeypoints.dir/build.make CMakeFiles/drawKeypoints.dir/build
.PHONY : drawKeypoints/fast

#=============================================================================
# Target rules for targets named drawKeypointsAll

# Build rule for target.
drawKeypointsAll: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 drawKeypointsAll
.PHONY : drawKeypointsAll

# fast build rule for target.
drawKeypointsAll/fast:
	$(MAKE) -f CMakeFiles/drawKeypointsAll.dir/build.make CMakeFiles/drawKeypointsAll.dir/build
.PHONY : drawKeypointsAll/fast

#=============================================================================
# Target rules for targets named drawKeypointsArch

# Build rule for target.
drawKeypointsArch: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 drawKeypointsArch
.PHONY : drawKeypointsArch

# fast build rule for target.
drawKeypointsArch/fast:
	$(MAKE) -f CMakeFiles/drawKeypointsArch.dir/build.make CMakeFiles/drawKeypointsArch.dir/build
.PHONY : drawKeypointsArch/fast

#=============================================================================
# Target rules for targets named extractSURF

# Build rule for target.
extractSURF: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 extractSURF
.PHONY : extractSURF

# fast build rule for target.
extractSURF/fast:
	$(MAKE) -f CMakeFiles/extractSURF.dir/build.make CMakeFiles/extractSURF.dir/build
.PHONY : extractSURF/fast

#=============================================================================
# Target rules for targets named findingCenter

# Build rule for target.
findingCenter: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 findingCenter
.PHONY : findingCenter

# fast build rule for target.
findingCenter/fast:
	$(MAKE) -f CMakeFiles/findingCenter.dir/build.make CMakeFiles/findingCenter.dir/build
.PHONY : findingCenter/fast

#=============================================================================
# Target rules for targets named image_view

# Build rule for target.
image_view: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 image_view
.PHONY : image_view

# fast build rule for target.
image_view/fast:
	$(MAKE) -f CMakeFiles/image_view.dir/build.make CMakeFiles/image_view.dir/build
.PHONY : image_view/fast

#=============================================================================
# Target rules for targets named rosbuild_precompile

# Build rule for target.
rosbuild_precompile: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_precompile
.PHONY : rosbuild_precompile

# fast build rule for target.
rosbuild_precompile/fast:
	$(MAKE) -f CMakeFiles/rosbuild_precompile.dir/build.make CMakeFiles/rosbuild_precompile.dir/build
.PHONY : rosbuild_precompile/fast

#=============================================================================
# Target rules for targets named rosbuild_premsgsrvgen

# Build rule for target.
rosbuild_premsgsrvgen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_premsgsrvgen
.PHONY : rosbuild_premsgsrvgen

# fast build rule for target.
rosbuild_premsgsrvgen/fast:
	$(MAKE) -f CMakeFiles/rosbuild_premsgsrvgen.dir/build.make CMakeFiles/rosbuild_premsgsrvgen.dir/build
.PHONY : rosbuild_premsgsrvgen/fast

#=============================================================================
# Target rules for targets named rospack_genmsg

# Build rule for target.
rospack_genmsg: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg
.PHONY : rospack_genmsg

# fast build rule for target.
rospack_genmsg/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg.dir/build.make CMakeFiles/rospack_genmsg.dir/build
.PHONY : rospack_genmsg/fast

#=============================================================================
# Target rules for targets named rospack_genmsg_all

# Build rule for target.
rospack_genmsg_all: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg_all
.PHONY : rospack_genmsg_all

# fast build rule for target.
rospack_genmsg_all/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg_all.dir/build.make CMakeFiles/rospack_genmsg_all.dir/build
.PHONY : rospack_genmsg_all/fast

#=============================================================================
# Target rules for targets named rospack_genmsg_libexe

# Build rule for target.
rospack_genmsg_libexe: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg_libexe
.PHONY : rospack_genmsg_libexe

# fast build rule for target.
rospack_genmsg_libexe/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg_libexe.dir/build.make CMakeFiles/rospack_genmsg_libexe.dir/build
.PHONY : rospack_genmsg_libexe/fast

#=============================================================================
# Target rules for targets named rospack_gensrv

# Build rule for target.
rospack_gensrv: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_gensrv
.PHONY : rospack_gensrv

# fast build rule for target.
rospack_gensrv/fast:
	$(MAKE) -f CMakeFiles/rospack_gensrv.dir/build.make CMakeFiles/rospack_gensrv.dir/build
.PHONY : rospack_gensrv/fast

#=============================================================================
# Target rules for targets named rospack_gensrv_all

# Build rule for target.
rospack_gensrv_all: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_gensrv_all
.PHONY : rospack_gensrv_all

# fast build rule for target.
rospack_gensrv_all/fast:
	$(MAKE) -f CMakeFiles/rospack_gensrv_all.dir/build.make CMakeFiles/rospack_gensrv_all.dir/build
.PHONY : rospack_gensrv_all/fast

#=============================================================================
# Target rules for targets named test

# Build rule for target.
test: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test
.PHONY : test

# fast build rule for target.
test/fast:
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/build
.PHONY : test/fast

#=============================================================================
# Target rules for targets named test-future

# Build rule for target.
test-future: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-future
.PHONY : test-future

# fast build rule for target.
test-future/fast:
	$(MAKE) -f CMakeFiles/test-future.dir/build.make CMakeFiles/test-future.dir/build
.PHONY : test-future/fast

#=============================================================================
# Target rules for targets named test-results

# Build rule for target.
test-results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-results
.PHONY : test-results

# fast build rule for target.
test-results/fast:
	$(MAKE) -f CMakeFiles/test-results.dir/build.make CMakeFiles/test-results.dir/build
.PHONY : test-results/fast

#=============================================================================
# Target rules for targets named test-results-run

# Build rule for target.
test-results-run: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-results-run
.PHONY : test-results-run

# fast build rule for target.
test-results-run/fast:
	$(MAKE) -f CMakeFiles/test-results-run.dir/build.make CMakeFiles/test-results-run.dir/build
.PHONY : test-results-run/fast

#=============================================================================
# Target rules for targets named tests

# Build rule for target.
tests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tests
.PHONY : tests

# fast build rule for target.
tests/fast:
	$(MAKE) -f CMakeFiles/tests.dir/build.make CMakeFiles/tests.dir/build
.PHONY : tests/fast

#=============================================================================
# Target rules for targets named tune_cropped

# Build rule for target.
tune_cropped: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tune_cropped
.PHONY : tune_cropped

# fast build rule for target.
tune_cropped/fast:
	$(MAKE) -f CMakeFiles/tune_cropped.dir/build.make CMakeFiles/tune_cropped.dir/build
.PHONY : tune_cropped/fast

#=============================================================================
# Target rules for targets named verify_object_service

# Build rule for target.
verify_object_service: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 verify_object_service
.PHONY : verify_object_service

# fast build rule for target.
verify_object_service/fast:
	$(MAKE) -f CMakeFiles/verify_object_service.dir/build.make CMakeFiles/verify_object_service.dir/build
.PHONY : verify_object_service/fast

# target to build an object file
src/drawKeypoints.o:
	$(MAKE) -f CMakeFiles/drawKeypoints.dir/build.make CMakeFiles/drawKeypoints.dir/src/drawKeypoints.o
.PHONY : src/drawKeypoints.o

# target to preprocess a source file
src/drawKeypoints.i:
	$(MAKE) -f CMakeFiles/drawKeypoints.dir/build.make CMakeFiles/drawKeypoints.dir/src/drawKeypoints.i
.PHONY : src/drawKeypoints.i

# target to generate assembly for a file
src/drawKeypoints.s:
	$(MAKE) -f CMakeFiles/drawKeypoints.dir/build.make CMakeFiles/drawKeypoints.dir/src/drawKeypoints.s
.PHONY : src/drawKeypoints.s

# target to build an object file
src/drawKeypointsAll.o:
	$(MAKE) -f CMakeFiles/drawKeypointsAll.dir/build.make CMakeFiles/drawKeypointsAll.dir/src/drawKeypointsAll.o
.PHONY : src/drawKeypointsAll.o

# target to preprocess a source file
src/drawKeypointsAll.i:
	$(MAKE) -f CMakeFiles/drawKeypointsAll.dir/build.make CMakeFiles/drawKeypointsAll.dir/src/drawKeypointsAll.i
.PHONY : src/drawKeypointsAll.i

# target to generate assembly for a file
src/drawKeypointsAll.s:
	$(MAKE) -f CMakeFiles/drawKeypointsAll.dir/build.make CMakeFiles/drawKeypointsAll.dir/src/drawKeypointsAll.s
.PHONY : src/drawKeypointsAll.s

# target to build an object file
src/drawKeypointsArch.o:
	$(MAKE) -f CMakeFiles/drawKeypointsArch.dir/build.make CMakeFiles/drawKeypointsArch.dir/src/drawKeypointsArch.o
.PHONY : src/drawKeypointsArch.o

# target to preprocess a source file
src/drawKeypointsArch.i:
	$(MAKE) -f CMakeFiles/drawKeypointsArch.dir/build.make CMakeFiles/drawKeypointsArch.dir/src/drawKeypointsArch.i
.PHONY : src/drawKeypointsArch.i

# target to generate assembly for a file
src/drawKeypointsArch.s:
	$(MAKE) -f CMakeFiles/drawKeypointsArch.dir/build.make CMakeFiles/drawKeypointsArch.dir/src/drawKeypointsArch.s
.PHONY : src/drawKeypointsArch.s

# target to build an object file
src/extractSURF.o:
	$(MAKE) -f CMakeFiles/extractSURF.dir/build.make CMakeFiles/extractSURF.dir/src/extractSURF.o
.PHONY : src/extractSURF.o

# target to preprocess a source file
src/extractSURF.i:
	$(MAKE) -f CMakeFiles/extractSURF.dir/build.make CMakeFiles/extractSURF.dir/src/extractSURF.i
.PHONY : src/extractSURF.i

# target to generate assembly for a file
src/extractSURF.s:
	$(MAKE) -f CMakeFiles/extractSURF.dir/build.make CMakeFiles/extractSURF.dir/src/extractSURF.s
.PHONY : src/extractSURF.s

# target to build an object file
src/findingCenter.o:
	$(MAKE) -f CMakeFiles/findingCenter.dir/build.make CMakeFiles/findingCenter.dir/src/findingCenter.o
.PHONY : src/findingCenter.o

# target to preprocess a source file
src/findingCenter.i:
	$(MAKE) -f CMakeFiles/findingCenter.dir/build.make CMakeFiles/findingCenter.dir/src/findingCenter.i
.PHONY : src/findingCenter.i

# target to generate assembly for a file
src/findingCenter.s:
	$(MAKE) -f CMakeFiles/findingCenter.dir/build.make CMakeFiles/findingCenter.dir/src/findingCenter.s
.PHONY : src/findingCenter.s

# target to build an object file
src/image_view.o:
	$(MAKE) -f CMakeFiles/image_view.dir/build.make CMakeFiles/image_view.dir/src/image_view.o
.PHONY : src/image_view.o

# target to preprocess a source file
src/image_view.i:
	$(MAKE) -f CMakeFiles/image_view.dir/build.make CMakeFiles/image_view.dir/src/image_view.i
.PHONY : src/image_view.i

# target to generate assembly for a file
src/image_view.s:
	$(MAKE) -f CMakeFiles/image_view.dir/build.make CMakeFiles/image_view.dir/src/image_view.s
.PHONY : src/image_view.s

# target to build an object file
src/tune_cropped.o:
	$(MAKE) -f CMakeFiles/tune_cropped.dir/build.make CMakeFiles/tune_cropped.dir/src/tune_cropped.o
.PHONY : src/tune_cropped.o

# target to preprocess a source file
src/tune_cropped.i:
	$(MAKE) -f CMakeFiles/tune_cropped.dir/build.make CMakeFiles/tune_cropped.dir/src/tune_cropped.i
.PHONY : src/tune_cropped.i

# target to generate assembly for a file
src/tune_cropped.s:
	$(MAKE) -f CMakeFiles/tune_cropped.dir/build.make CMakeFiles/tune_cropped.dir/src/tune_cropped.s
.PHONY : src/tune_cropped.s

# target to build an object file
src/verify_object.o:
	$(MAKE) -f CMakeFiles/verify_object_service.dir/build.make CMakeFiles/verify_object_service.dir/src/verify_object.o
.PHONY : src/verify_object.o

# target to preprocess a source file
src/verify_object.i:
	$(MAKE) -f CMakeFiles/verify_object_service.dir/build.make CMakeFiles/verify_object_service.dir/src/verify_object.i
.PHONY : src/verify_object.i

# target to generate assembly for a file
src/verify_object.s:
	$(MAKE) -f CMakeFiles/verify_object_service.dir/build.make CMakeFiles/verify_object_service.dir/src/verify_object.s
.PHONY : src/verify_object.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... ROSBUILD_genmsg_cpp"
	@echo "... ROSBUILD_genmsg_lisp"
	@echo "... ROSBUILD_genmsg_py"
	@echo "... ROSBUILD_gensrv_cpp"
	@echo "... ROSBUILD_gensrv_lisp"
	@echo "... ROSBUILD_gensrv_py"
	@echo "... clean-test-results"
	@echo "... drawKeypoints"
	@echo "... drawKeypointsAll"
	@echo "... drawKeypointsArch"
	@echo "... edit_cache"
	@echo "... extractSURF"
	@echo "... findingCenter"
	@echo "... image_view"
	@echo "... rebuild_cache"
	@echo "... rosbuild_precompile"
	@echo "... rosbuild_premsgsrvgen"
	@echo "... rospack_genmsg"
	@echo "... rospack_genmsg_all"
	@echo "... rospack_genmsg_libexe"
	@echo "... rospack_gensrv"
	@echo "... rospack_gensrv_all"
	@echo "... test"
	@echo "... test-future"
	@echo "... test-results"
	@echo "... test-results-run"
	@echo "... tests"
	@echo "... tune_cropped"
	@echo "... verify_object_service"
	@echo "... src/drawKeypoints.o"
	@echo "... src/drawKeypoints.i"
	@echo "... src/drawKeypoints.s"
	@echo "... src/drawKeypointsAll.o"
	@echo "... src/drawKeypointsAll.i"
	@echo "... src/drawKeypointsAll.s"
	@echo "... src/drawKeypointsArch.o"
	@echo "... src/drawKeypointsArch.i"
	@echo "... src/drawKeypointsArch.s"
	@echo "... src/extractSURF.o"
	@echo "... src/extractSURF.i"
	@echo "... src/extractSURF.s"
	@echo "... src/findingCenter.o"
	@echo "... src/findingCenter.i"
	@echo "... src/findingCenter.s"
	@echo "... src/image_view.o"
	@echo "... src/image_view.i"
	@echo "... src/image_view.s"
	@echo "... src/tune_cropped.o"
	@echo "... src/tune_cropped.i"
	@echo "... src/tune_cropped.s"
	@echo "... src/verify_object.o"
	@echo "... src/verify_object.i"
	@echo "... src/verify_object.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

