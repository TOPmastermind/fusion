# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build

# Include any dependencies generated for this target.
include CMakeFiles/pinhole.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pinhole.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pinhole.dir/flags.make

CMakeFiles/pinhole.dir/pinhole.cpp.o: CMakeFiles/pinhole.dir/flags.make
CMakeFiles/pinhole.dir/pinhole.cpp.o: ../pinhole.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pinhole.dir/pinhole.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/pinhole.dir/pinhole.cpp.o -c /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/pinhole.cpp

CMakeFiles/pinhole.dir/pinhole.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pinhole.dir/pinhole.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/pinhole.cpp > CMakeFiles/pinhole.dir/pinhole.cpp.i

CMakeFiles/pinhole.dir/pinhole.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pinhole.dir/pinhole.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/pinhole.cpp -o CMakeFiles/pinhole.dir/pinhole.cpp.s

CMakeFiles/pinhole.dir/pinhole.cpp.o.requires:

.PHONY : CMakeFiles/pinhole.dir/pinhole.cpp.o.requires

CMakeFiles/pinhole.dir/pinhole.cpp.o.provides: CMakeFiles/pinhole.dir/pinhole.cpp.o.requires
	$(MAKE) -f CMakeFiles/pinhole.dir/build.make CMakeFiles/pinhole.dir/pinhole.cpp.o.provides.build
.PHONY : CMakeFiles/pinhole.dir/pinhole.cpp.o.provides

CMakeFiles/pinhole.dir/pinhole.cpp.o.provides.build: CMakeFiles/pinhole.dir/pinhole.cpp.o


# Object files for target pinhole
pinhole_OBJECTS = \
"CMakeFiles/pinhole.dir/pinhole.cpp.o"

# External object files for target pinhole
pinhole_EXTERNAL_OBJECTS =

pinhole: CMakeFiles/pinhole.dir/pinhole.cpp.o
pinhole: CMakeFiles/pinhole.dir/build.make
pinhole: /opt/local/lib/libboost_system-mt.dylib
pinhole: /opt/local/lib/libboost_filesystem-mt.dylib
pinhole: /opt/local/lib/libboost_thread-mt.dylib
pinhole: /opt/local/lib/libboost_date_time-mt.dylib
pinhole: /opt/local/lib/libboost_iostreams-mt.dylib
pinhole: /opt/local/lib/libboost_serialization-mt.dylib
pinhole: /usr/local/lib/libpcl_common.dylib
pinhole: /usr/local/lib/libpcl_octree.dylib
pinhole: /usr/local/lib/libpcl_io.dylib
pinhole: /opt/local/lib/libflann_cpp_s.a
pinhole: /usr/local/lib/libpcl_kdtree.dylib
pinhole: /usr/local/lib/libpcl_search.dylib
pinhole: /usr/local/lib/libpcl_sample_consensus.dylib
pinhole: /usr/local/lib/libpcl_filters.dylib
pinhole: /usr/local/lib/libpcl_features.dylib
pinhole: /usr/local/lib/libpcl_ml.dylib
pinhole: /usr/local/lib/libpcl_segmentation.dylib
pinhole: /usr/local/lib/libpcl_visualization.dylib
pinhole: /opt/local/lib/libqhull_p.dylib
pinhole: /usr/local/lib/libpcl_surface.dylib
pinhole: /usr/local/lib/libpcl_registration.dylib
pinhole: /usr/local/lib/libpcl_keypoints.dylib
pinhole: /usr/local/lib/libpcl_tracking.dylib
pinhole: /usr/local/lib/libpcl_recognition.dylib
pinhole: /usr/local/lib/libpcl_stereo.dylib
pinhole: /usr/local/lib/libpcl_outofcore.dylib
pinhole: /usr/local/lib/libpcl_people.dylib
pinhole: /opt/local/lib/libboost_system-mt.dylib
pinhole: /opt/local/lib/libboost_filesystem-mt.dylib
pinhole: /opt/local/lib/libboost_thread-mt.dylib
pinhole: /opt/local/lib/libboost_date_time-mt.dylib
pinhole: /opt/local/lib/libboost_iostreams-mt.dylib
pinhole: /opt/local/lib/libboost_serialization-mt.dylib
pinhole: /opt/local/lib/libqhull_p.dylib
pinhole: /opt/local/lib/libflann_cpp_s.a
pinhole: /usr/local/lib/libvtkDomainsChemistry-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersFlowPaths-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersGeneric-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersHyperTree-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersParallelImaging-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersProgrammable-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersSelection-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersSMP-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersTexture-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersVerdict-6.3.1.dylib
pinhole: /usr/local/lib/libvtkverdict-6.3.1.dylib
pinhole: /usr/local/lib/libvtkGeovisCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkproj4-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingMath-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingMorphological-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingStatistics-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingStencil-6.3.1.dylib
pinhole: /usr/local/lib/libvtkInteractionImage-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOAMR-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOEnSight-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOExodus-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOExport-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingGL2PS-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingContextOpenGL-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOImport-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOInfovis-6.3.1.dylib
pinhole: /usr/local/lib/libvtklibxml2-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOLSDyna-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOMINC-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOMovie-6.3.1.dylib
pinhole: /usr/local/lib/libvtkoggtheora-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOParallel-6.3.1.dylib
pinhole: /usr/local/lib/libvtkjsoncpp-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOParallelXML-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOPLY-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOSQL-6.3.1.dylib
pinhole: /usr/local/lib/libvtksqlite-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOVideo-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingImage-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingLIC-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingLOD-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.3.1.dylib
pinhole: /usr/local/lib/libvtkViewsContext2D-6.3.1.dylib
pinhole: /usr/local/lib/libvtkViewsInfovis-6.3.1.dylib
pinhole: /usr/local/lib/libpcl_common.dylib
pinhole: /usr/local/lib/libpcl_octree.dylib
pinhole: /usr/local/lib/libpcl_io.dylib
pinhole: /usr/local/lib/libpcl_kdtree.dylib
pinhole: /usr/local/lib/libpcl_search.dylib
pinhole: /usr/local/lib/libpcl_sample_consensus.dylib
pinhole: /usr/local/lib/libpcl_filters.dylib
pinhole: /usr/local/lib/libpcl_features.dylib
pinhole: /usr/local/lib/libpcl_ml.dylib
pinhole: /usr/local/lib/libpcl_segmentation.dylib
pinhole: /usr/local/lib/libpcl_visualization.dylib
pinhole: /usr/local/lib/libpcl_surface.dylib
pinhole: /usr/local/lib/libpcl_registration.dylib
pinhole: /usr/local/lib/libpcl_keypoints.dylib
pinhole: /usr/local/lib/libpcl_tracking.dylib
pinhole: /usr/local/lib/libpcl_recognition.dylib
pinhole: /usr/local/lib/libpcl_stereo.dylib
pinhole: /usr/local/lib/libpcl_outofcore.dylib
pinhole: /usr/local/lib/libpcl_people.dylib
pinhole: /usr/local/lib/libvtkFiltersAMR-6.3.1.dylib
pinhole: /usr/local/lib/libvtkgl2ps-6.3.1.dylib
pinhole: /usr/local/lib/libvtkexoIIc-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersParallel-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIONetCDF-6.3.1.dylib
pinhole: /usr/local/lib/libvtkNetCDF_cxx-6.3.1.dylib
pinhole: /usr/local/lib/libvtkNetCDF-6.3.1.dylib
pinhole: /usr/local/lib/libvtkhdf5_hl-6.3.1.dylib
pinhole: /usr/local/lib/libvtkhdf5-6.3.1.dylib
pinhole: /usr/local/lib/libvtkParallelCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOXML-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOGeometry-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOXMLParser-6.3.1.dylib
pinhole: /usr/local/lib/libvtkexpat-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOLegacy-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingOpenGL-6.3.1.dylib
pinhole: /usr/local/lib/libvtkChartsCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingContext2D-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersImaging-6.3.1.dylib
pinhole: /usr/local/lib/libvtkInfovisLayout-6.3.1.dylib
pinhole: /usr/local/lib/libvtkInfovisCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkViewsCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkInteractionWidgets-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersHybrid-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingGeneral-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingSources-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersModeling-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingHybrid-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOImage-6.3.1.dylib
pinhole: /usr/local/lib/libvtkDICOMParser-6.3.1.dylib
pinhole: /usr/local/lib/libvtkIOCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkmetaio-6.3.1.dylib
pinhole: /usr/local/lib/libvtkpng-6.3.1.dylib
pinhole: /usr/local/lib/libvtktiff-6.3.1.dylib
pinhole: /usr/local/lib/libvtkjpeg-6.3.1.dylib
pinhole: /usr/local/lib/libvtkInteractionStyle-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingAnnotation-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingColor-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingVolume-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingLabel-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingFreeType-6.3.1.dylib
pinhole: /usr/local/lib/libvtkRenderingCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonColor-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersExtraction-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersStatistics-6.3.1.dylib
pinhole: /usr/local/lib/libvtkalglib-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingFourier-6.3.1.dylib
pinhole: /usr/local/lib/libvtkImagingCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersGeometry-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersSources-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersGeneral-6.3.1.dylib
pinhole: /usr/local/lib/libvtkFiltersCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonExecutionModel-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonComputationalGeometry-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonDataModel-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonMisc-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonTransforms-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonMath-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonSystem-6.3.1.dylib
pinhole: /usr/local/lib/libvtkCommonCore-6.3.1.dylib
pinhole: /usr/local/lib/libvtksys-6.3.1.dylib
pinhole: /usr/local/lib/libvtkftgl-6.3.1.dylib
pinhole: /usr/local/lib/libvtkfreetype-6.3.1.dylib
pinhole: /usr/local/lib/libvtkzlib-6.3.1.dylib
pinhole: CMakeFiles/pinhole.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable pinhole"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pinhole.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pinhole.dir/build: pinhole

.PHONY : CMakeFiles/pinhole.dir/build

CMakeFiles/pinhole.dir/requires: CMakeFiles/pinhole.dir/pinhole.cpp.o.requires

.PHONY : CMakeFiles/pinhole.dir/requires

CMakeFiles/pinhole.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pinhole.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pinhole.dir/clean

CMakeFiles/pinhole.dir/depend:
	cd /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5 /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5 /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build/CMakeFiles/pinhole.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pinhole.dir/depend
