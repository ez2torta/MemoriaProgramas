# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform

# Include any dependencies generated for this target.
include CMakeFiles/transform.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/transform.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/transform.dir/flags.make

CMakeFiles/transform.dir/transform.cpp.o: CMakeFiles/transform.dir/flags.make
CMakeFiles/transform.dir/transform.cpp.o: transform.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/transform.dir/transform.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/transform.dir/transform.cpp.o -c /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform/transform.cpp

CMakeFiles/transform.dir/transform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/transform.dir/transform.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform/transform.cpp > CMakeFiles/transform.dir/transform.cpp.i

CMakeFiles/transform.dir/transform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/transform.dir/transform.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform/transform.cpp -o CMakeFiles/transform.dir/transform.cpp.s

CMakeFiles/transform.dir/transform.cpp.o.requires:
.PHONY : CMakeFiles/transform.dir/transform.cpp.o.requires

CMakeFiles/transform.dir/transform.cpp.o.provides: CMakeFiles/transform.dir/transform.cpp.o.requires
	$(MAKE) -f CMakeFiles/transform.dir/build.make CMakeFiles/transform.dir/transform.cpp.o.provides.build
.PHONY : CMakeFiles/transform.dir/transform.cpp.o.provides

CMakeFiles/transform.dir/transform.cpp.o.provides.build: CMakeFiles/transform.dir/transform.cpp.o

# Object files for target transform
transform_OBJECTS = \
"CMakeFiles/transform.dir/transform.cpp.o"

# External object files for target transform
transform_EXTERNAL_OBJECTS =

transform: CMakeFiles/transform.dir/transform.cpp.o
transform: CMakeFiles/transform.dir/build.make
transform: /usr/lib/x86_64-linux-gnu/libboost_system.so
transform: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
transform: /usr/lib/x86_64-linux-gnu/libboost_thread.so
transform: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
transform: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
transform: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
transform: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
transform: /usr/lib/x86_64-linux-gnu/libpthread.so
transform: /usr/lib/libpcl_common.so
transform: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
transform: /usr/lib/libpcl_kdtree.so
transform: /usr/lib/libpcl_octree.so
transform: /usr/lib/libpcl_search.so
transform: /usr/lib/x86_64-linux-gnu/libqhull.so
transform: /usr/lib/libpcl_surface.so
transform: /usr/lib/libpcl_sample_consensus.so
transform: /usr/lib/libOpenNI.so
transform: /usr/lib/libOpenNI2.so
transform: /usr/lib/libvtkCommon.so.5.8.0
transform: /usr/lib/libvtkFiltering.so.5.8.0
transform: /usr/lib/libvtkImaging.so.5.8.0
transform: /usr/lib/libvtkGraphics.so.5.8.0
transform: /usr/lib/libvtkGenericFiltering.so.5.8.0
transform: /usr/lib/libvtkIO.so.5.8.0
transform: /usr/lib/libvtkRendering.so.5.8.0
transform: /usr/lib/libvtkVolumeRendering.so.5.8.0
transform: /usr/lib/libvtkHybrid.so.5.8.0
transform: /usr/lib/libvtkWidgets.so.5.8.0
transform: /usr/lib/libvtkParallel.so.5.8.0
transform: /usr/lib/libvtkInfovis.so.5.8.0
transform: /usr/lib/libvtkGeovis.so.5.8.0
transform: /usr/lib/libvtkViews.so.5.8.0
transform: /usr/lib/libvtkCharts.so.5.8.0
transform: /usr/lib/libpcl_io.so
transform: /usr/lib/libpcl_filters.so
transform: /usr/lib/libpcl_features.so
transform: /usr/lib/libpcl_keypoints.so
transform: /usr/lib/libpcl_registration.so
transform: /usr/lib/libpcl_segmentation.so
transform: /usr/lib/libpcl_recognition.so
transform: /usr/lib/libpcl_visualization.so
transform: /usr/lib/libpcl_people.so
transform: /usr/lib/libpcl_outofcore.so
transform: /usr/lib/libpcl_tracking.so
transform: /usr/lib/libpcl_apps.so
transform: /usr/lib/x86_64-linux-gnu/libboost_system.so
transform: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
transform: /usr/lib/x86_64-linux-gnu/libboost_thread.so
transform: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
transform: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
transform: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
transform: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
transform: /usr/lib/x86_64-linux-gnu/libpthread.so
transform: /usr/lib/x86_64-linux-gnu/libqhull.so
transform: /usr/lib/libOpenNI.so
transform: /usr/lib/libOpenNI2.so
transform: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
transform: /usr/lib/libvtkCommon.so.5.8.0
transform: /usr/lib/libvtkFiltering.so.5.8.0
transform: /usr/lib/libvtkImaging.so.5.8.0
transform: /usr/lib/libvtkGraphics.so.5.8.0
transform: /usr/lib/libvtkGenericFiltering.so.5.8.0
transform: /usr/lib/libvtkIO.so.5.8.0
transform: /usr/lib/libvtkRendering.so.5.8.0
transform: /usr/lib/libvtkVolumeRendering.so.5.8.0
transform: /usr/lib/libvtkHybrid.so.5.8.0
transform: /usr/lib/libvtkWidgets.so.5.8.0
transform: /usr/lib/libvtkParallel.so.5.8.0
transform: /usr/lib/libvtkInfovis.so.5.8.0
transform: /usr/lib/libvtkGeovis.so.5.8.0
transform: /usr/lib/libvtkViews.so.5.8.0
transform: /usr/lib/libvtkCharts.so.5.8.0
transform: /usr/lib/libpcl_common.so
transform: /usr/lib/libpcl_kdtree.so
transform: /usr/lib/libpcl_octree.so
transform: /usr/lib/libpcl_search.so
transform: /usr/lib/libpcl_surface.so
transform: /usr/lib/libpcl_sample_consensus.so
transform: /usr/lib/libpcl_io.so
transform: /usr/lib/libpcl_filters.so
transform: /usr/lib/libpcl_features.so
transform: /usr/lib/libpcl_keypoints.so
transform: /usr/lib/libpcl_registration.so
transform: /usr/lib/libpcl_segmentation.so
transform: /usr/lib/libpcl_recognition.so
transform: /usr/lib/libpcl_visualization.so
transform: /usr/lib/libpcl_people.so
transform: /usr/lib/libpcl_outofcore.so
transform: /usr/lib/libpcl_tracking.so
transform: /usr/lib/libpcl_apps.so
transform: /usr/lib/libvtkViews.so.5.8.0
transform: /usr/lib/libvtkInfovis.so.5.8.0
transform: /usr/lib/libvtkWidgets.so.5.8.0
transform: /usr/lib/libvtkVolumeRendering.so.5.8.0
transform: /usr/lib/libvtkHybrid.so.5.8.0
transform: /usr/lib/libvtkParallel.so.5.8.0
transform: /usr/lib/libvtkRendering.so.5.8.0
transform: /usr/lib/libvtkImaging.so.5.8.0
transform: /usr/lib/libvtkGraphics.so.5.8.0
transform: /usr/lib/libvtkIO.so.5.8.0
transform: /usr/lib/libvtkFiltering.so.5.8.0
transform: /usr/lib/libvtkCommon.so.5.8.0
transform: /usr/lib/libvtksys.so.5.8.0
transform: CMakeFiles/transform.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable transform"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/transform.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/transform.dir/build: transform
.PHONY : CMakeFiles/transform.dir/build

CMakeFiles/transform.dir/requires: CMakeFiles/transform.dir/transform.cpp.o.requires
.PHONY : CMakeFiles/transform.dir/requires

CMakeFiles/transform.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/transform.dir/cmake_clean.cmake
.PHONY : CMakeFiles/transform.dir/clean

CMakeFiles/transform.dir/depend:
	cd /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform /home/osboxes/Develop/Memoria/MemoriaProgramas/Transform/CMakeFiles/transform.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/transform.dir/depend

