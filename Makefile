# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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
CMAKE_SOURCE_DIR = /home/skuba-athome/skuba_athome_main/objects

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/skuba-athome/skuba_athome_main/objects

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
	$(CMAKE_COMMAND) -E cmake_progress_start /home/skuba-athome/skuba_athome_main/objects/CMakeFiles /home/skuba-athome/skuba_athome_main/objects/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/skuba-athome/skuba_athome_main/objects/CMakeFiles 0
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
# Target rules for targets named ece

# Build rule for target.
ece: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ece
.PHONY : ece

# fast build rule for target.
ece/fast:
	$(MAKE) -f CMakeFiles/ece.dir/build.make CMakeFiles/ece.dir/build
.PHONY : ece/fast

#=============================================================================
# Target rules for targets named main

# Build rule for target.
main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 main
.PHONY : main

# fast build rule for target.
main/fast:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/build
.PHONY : main/fast

#=============================================================================
# Target rules for targets named nearest_neighbors

# Build rule for target.
nearest_neighbors: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 nearest_neighbors
.PHONY : nearest_neighbors

# fast build rule for target.
nearest_neighbors/fast:
	$(MAKE) -f CMakeFiles/nearest_neighbors.dir/build.make CMakeFiles/nearest_neighbors.dir/build
.PHONY : nearest_neighbors/fast

#=============================================================================
# Target rules for targets named open_door

# Build rule for target.
open_door: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 open_door
.PHONY : open_door

# fast build rule for target.
open_door/fast:
	$(MAKE) -f CMakeFiles/open_door.dir/build.make CMakeFiles/open_door.dir/build
.PHONY : open_door/fast

#=============================================================================
# Target rules for targets named pcd_viewer

# Build rule for target.
pcd_viewer: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 pcd_viewer
.PHONY : pcd_viewer

# fast build rule for target.
pcd_viewer/fast:
	$(MAKE) -f CMakeFiles/pcd_viewer.dir/build.make CMakeFiles/pcd_viewer.dir/build
.PHONY : pcd_viewer/fast

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
# Target rules for targets named tester

# Build rule for target.
tester: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tester
.PHONY : tester

# fast build rule for target.
tester/fast:
	$(MAKE) -f CMakeFiles/tester.dir/build.make CMakeFiles/tester.dir/build
.PHONY : tester/fast

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

# target to build an object file
src/ece.o:
	$(MAKE) -f CMakeFiles/ece.dir/build.make CMakeFiles/ece.dir/src/ece.o
.PHONY : src/ece.o

# target to preprocess a source file
src/ece.i:
	$(MAKE) -f CMakeFiles/ece.dir/build.make CMakeFiles/ece.dir/src/ece.i
.PHONY : src/ece.i

# target to generate assembly for a file
src/ece.s:
	$(MAKE) -f CMakeFiles/ece.dir/build.make CMakeFiles/ece.dir/src/ece.s
.PHONY : src/ece.s

# target to build an object file
src/main.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/main.o
.PHONY : src/main.o

# target to preprocess a source file
src/main.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/main.i
.PHONY : src/main.i

# target to generate assembly for a file
src/main.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/main.s
.PHONY : src/main.s

# target to build an object file
src/nearest_neighbors.o:
	$(MAKE) -f CMakeFiles/nearest_neighbors.dir/build.make CMakeFiles/nearest_neighbors.dir/src/nearest_neighbors.o
.PHONY : src/nearest_neighbors.o

# target to preprocess a source file
src/nearest_neighbors.i:
	$(MAKE) -f CMakeFiles/nearest_neighbors.dir/build.make CMakeFiles/nearest_neighbors.dir/src/nearest_neighbors.i
.PHONY : src/nearest_neighbors.i

# target to generate assembly for a file
src/nearest_neighbors.s:
	$(MAKE) -f CMakeFiles/nearest_neighbors.dir/build.make CMakeFiles/nearest_neighbors.dir/src/nearest_neighbors.s
.PHONY : src/nearest_neighbors.s

# target to build an object file
src/open_door.o:
	$(MAKE) -f CMakeFiles/open_door.dir/build.make CMakeFiles/open_door.dir/src/open_door.o
.PHONY : src/open_door.o

# target to preprocess a source file
src/open_door.i:
	$(MAKE) -f CMakeFiles/open_door.dir/build.make CMakeFiles/open_door.dir/src/open_door.i
.PHONY : src/open_door.i

# target to generate assembly for a file
src/open_door.s:
	$(MAKE) -f CMakeFiles/open_door.dir/build.make CMakeFiles/open_door.dir/src/open_door.s
.PHONY : src/open_door.s

# target to build an object file
src/pcd_viewer.o:
	$(MAKE) -f CMakeFiles/pcd_viewer.dir/build.make CMakeFiles/pcd_viewer.dir/src/pcd_viewer.o
.PHONY : src/pcd_viewer.o

# target to preprocess a source file
src/pcd_viewer.i:
	$(MAKE) -f CMakeFiles/pcd_viewer.dir/build.make CMakeFiles/pcd_viewer.dir/src/pcd_viewer.i
.PHONY : src/pcd_viewer.i

# target to generate assembly for a file
src/pcd_viewer.s:
	$(MAKE) -f CMakeFiles/pcd_viewer.dir/build.make CMakeFiles/pcd_viewer.dir/src/pcd_viewer.s
.PHONY : src/pcd_viewer.s

# target to build an object file
src/test.o:
	$(MAKE) -f CMakeFiles/tester.dir/build.make CMakeFiles/tester.dir/src/test.o
.PHONY : src/test.o

# target to preprocess a source file
src/test.i:
	$(MAKE) -f CMakeFiles/tester.dir/build.make CMakeFiles/tester.dir/src/test.i
.PHONY : src/test.i

# target to generate assembly for a file
src/test.s:
	$(MAKE) -f CMakeFiles/tester.dir/build.make CMakeFiles/tester.dir/src/test.s
.PHONY : src/test.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... ROSBUILD_genmsg_cpp"
	@echo "... ROSBUILD_genmsg_lisp"
	@echo "... ROSBUILD_gensrv_cpp"
	@echo "... ROSBUILD_gensrv_lisp"
	@echo "... clean-test-results"
	@echo "... ece"
	@echo "... edit_cache"
	@echo "... main"
	@echo "... nearest_neighbors"
	@echo "... open_door"
	@echo "... pcd_viewer"
	@echo "... rebuild_cache"
	@echo "... rosbuild_precompile"
	@echo "... rosbuild_premsgsrvgen"
	@echo "... rospack_genmsg"
	@echo "... rospack_genmsg_libexe"
	@echo "... rospack_gensrv"
	@echo "... test"
	@echo "... test-future"
	@echo "... test-results"
	@echo "... test-results-run"
	@echo "... tester"
	@echo "... tests"
	@echo "... src/ece.o"
	@echo "... src/ece.i"
	@echo "... src/ece.s"
	@echo "... src/main.o"
	@echo "... src/main.i"
	@echo "... src/main.s"
	@echo "... src/nearest_neighbors.o"
	@echo "... src/nearest_neighbors.i"
	@echo "... src/nearest_neighbors.s"
	@echo "... src/open_door.o"
	@echo "... src/open_door.i"
	@echo "... src/open_door.s"
	@echo "... src/pcd_viewer.o"
	@echo "... src/pcd_viewer.i"
	@echo "... src/pcd_viewer.s"
	@echo "... src/test.o"
	@echo "... src/test.i"
	@echo "... src/test.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

