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
include CMakeFiles/circle.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/circle.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/circle.dir/flags.make

CMakeFiles/circle.dir/circle.cpp.o: CMakeFiles/circle.dir/flags.make
CMakeFiles/circle.dir/circle.cpp.o: ../circle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/circle.dir/circle.cpp.o"
	/Library/Developer/CommandLineTools/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/circle.dir/circle.cpp.o -c /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/circle.cpp

CMakeFiles/circle.dir/circle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/circle.dir/circle.cpp.i"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/circle.cpp > CMakeFiles/circle.dir/circle.cpp.i

CMakeFiles/circle.dir/circle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/circle.dir/circle.cpp.s"
	/Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/circle.cpp -o CMakeFiles/circle.dir/circle.cpp.s

CMakeFiles/circle.dir/circle.cpp.o.requires:

.PHONY : CMakeFiles/circle.dir/circle.cpp.o.requires

CMakeFiles/circle.dir/circle.cpp.o.provides: CMakeFiles/circle.dir/circle.cpp.o.requires
	$(MAKE) -f CMakeFiles/circle.dir/build.make CMakeFiles/circle.dir/circle.cpp.o.provides.build
.PHONY : CMakeFiles/circle.dir/circle.cpp.o.provides

CMakeFiles/circle.dir/circle.cpp.o.provides.build: CMakeFiles/circle.dir/circle.cpp.o


# Object files for target circle
circle_OBJECTS = \
"CMakeFiles/circle.dir/circle.cpp.o"

# External object files for target circle
circle_EXTERNAL_OBJECTS =

circle: CMakeFiles/circle.dir/circle.cpp.o
circle: CMakeFiles/circle.dir/build.make
circle: /opt/local/lib/libboost_system-mt.dylib
circle: /opt/local/lib/libboost_filesystem-mt.dylib
circle: /opt/local/lib/libboost_thread-mt.dylib
circle: /opt/local/lib/libboost_date_time-mt.dylib
circle: /opt/local/lib/libboost_iostreams-mt.dylib
circle: /opt/local/lib/libboost_serialization-mt.dylib
circle: /usr/local/lib/libpcl_common.dylib
circle: /usr/local/lib/libpcl_octree.dylib
circle: /usr/local/lib/libpcl_io.dylib
circle: /opt/local/lib/libflann_cpp_s.a
circle: /usr/local/lib/libpcl_kdtree.dylib
circle: /usr/local/lib/libpcl_search.dylib
circle: /usr/local/lib/libpcl_sample_consensus.dylib
circle: /usr/local/lib/libpcl_filters.dylib
circle: /usr/local/lib/libpcl_features.dylib
circle: /usr/local/lib/libpcl_ml.dylib
circle: /usr/local/lib/libpcl_segmentation.dylib
circle: /usr/local/lib/libpcl_visualization.dylib
circle: /opt/local/lib/libqhull_p.dylib
circle: /usr/local/lib/libpcl_surface.dylib
circle: /usr/local/lib/libpcl_registration.dylib
circle: /usr/local/lib/libpcl_keypoints.dylib
circle: /usr/local/lib/libpcl_tracking.dylib
circle: /usr/local/lib/libpcl_recognition.dylib
circle: /usr/local/lib/libpcl_stereo.dylib
circle: /usr/local/lib/libpcl_outofcore.dylib
circle: /usr/local/lib/libpcl_people.dylib
circle: /opt/local/lib/libboost_system-mt.dylib
circle: /opt/local/lib/libboost_filesystem-mt.dylib
circle: /opt/local/lib/libboost_thread-mt.dylib
circle: /opt/local/lib/libboost_date_time-mt.dylib
circle: /opt/local/lib/libboost_iostreams-mt.dylib
circle: /opt/local/lib/libboost_serialization-mt.dylib
circle: /opt/local/lib/libqhull_p.dylib
circle: /opt/local/lib/libflann_cpp_s.a
circle: /usr/local/lib/libvtkDomainsChemistry-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersFlowPaths-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersGeneric-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersHyperTree-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersParallelImaging-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersProgrammable-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersSelection-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersSMP-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersTexture-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersVerdict-6.3.1.dylib
circle: /usr/local/lib/libvtkverdict-6.3.1.dylib
circle: /usr/local/lib/libvtkGeovisCore-6.3.1.dylib
circle: /usr/local/lib/libvtkproj4-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingMath-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingMorphological-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingStatistics-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingStencil-6.3.1.dylib
circle: /usr/local/lib/libvtkInteractionImage-6.3.1.dylib
circle: /usr/local/lib/libvtkIOAMR-6.3.1.dylib
circle: /usr/local/lib/libvtkIOEnSight-6.3.1.dylib
circle: /usr/local/lib/libvtkIOExodus-6.3.1.dylib
circle: /usr/local/lib/libvtkIOExport-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingGL2PS-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingContextOpenGL-6.3.1.dylib
circle: /usr/local/lib/libvtkIOImport-6.3.1.dylib
circle: /usr/local/lib/libvtkIOInfovis-6.3.1.dylib
circle: /usr/local/lib/libvtklibxml2-6.3.1.dylib
circle: /usr/local/lib/libvtkIOLSDyna-6.3.1.dylib
circle: /usr/local/lib/libvtkIOMINC-6.3.1.dylib
circle: /usr/local/lib/libvtkIOMovie-6.3.1.dylib
circle: /usr/local/lib/libvtkoggtheora-6.3.1.dylib
circle: /usr/local/lib/libvtkIOParallel-6.3.1.dylib
circle: /usr/local/lib/libvtkjsoncpp-6.3.1.dylib
circle: /usr/local/lib/libvtkIOParallelXML-6.3.1.dylib
circle: /usr/local/lib/libvtkIOPLY-6.3.1.dylib
circle: /usr/local/lib/libvtkIOSQL-6.3.1.dylib
circle: /usr/local/lib/libvtksqlite-6.3.1.dylib
circle: /usr/local/lib/libvtkIOVideo-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingImage-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingLIC-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingLOD-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.3.1.dylib
circle: /usr/local/lib/libvtkViewsContext2D-6.3.1.dylib
circle: /usr/local/lib/libvtkViewsInfovis-6.3.1.dylib
circle: /usr/local/lib/libpcl_common.dylib
circle: /usr/local/lib/libpcl_octree.dylib
circle: /usr/local/lib/libpcl_io.dylib
circle: /usr/local/lib/libpcl_kdtree.dylib
circle: /usr/local/lib/libpcl_search.dylib
circle: /usr/local/lib/libpcl_sample_consensus.dylib
circle: /usr/local/lib/libpcl_filters.dylib
circle: /usr/local/lib/libpcl_features.dylib
circle: /usr/local/lib/libpcl_ml.dylib
circle: /usr/local/lib/libpcl_segmentation.dylib
circle: /usr/local/lib/libpcl_visualization.dylib
circle: /usr/local/lib/libpcl_surface.dylib
circle: /usr/local/lib/libpcl_registration.dylib
circle: /usr/local/lib/libpcl_keypoints.dylib
circle: /usr/local/lib/libpcl_tracking.dylib
circle: /usr/local/lib/libpcl_recognition.dylib
circle: /usr/local/lib/libpcl_stereo.dylib
circle: /usr/local/lib/libpcl_outofcore.dylib
circle: /usr/local/lib/libpcl_people.dylib
circle: /usr/local/lib/libvtkFiltersAMR-6.3.1.dylib
circle: /usr/local/lib/libvtkgl2ps-6.3.1.dylib
circle: /usr/local/lib/libvtkexoIIc-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersParallel-6.3.1.dylib
circle: /usr/local/lib/libvtkIONetCDF-6.3.1.dylib
circle: /usr/local/lib/libvtkNetCDF_cxx-6.3.1.dylib
circle: /usr/local/lib/libvtkNetCDF-6.3.1.dylib
circle: /usr/local/lib/libvtkhdf5_hl-6.3.1.dylib
circle: /usr/local/lib/libvtkhdf5-6.3.1.dylib
circle: /usr/local/lib/libvtkParallelCore-6.3.1.dylib
circle: /usr/local/lib/libvtkIOXML-6.3.1.dylib
circle: /usr/local/lib/libvtkIOGeometry-6.3.1.dylib
circle: /usr/local/lib/libvtkIOXMLParser-6.3.1.dylib
circle: /usr/local/lib/libvtkexpat-6.3.1.dylib
circle: /usr/local/lib/libvtkIOLegacy-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingOpenGL-6.3.1.dylib
circle: /usr/local/lib/libvtkChartsCore-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingContext2D-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersImaging-6.3.1.dylib
circle: /usr/local/lib/libvtkInfovisLayout-6.3.1.dylib
circle: /usr/local/lib/libvtkInfovisCore-6.3.1.dylib
circle: /usr/local/lib/libvtkViewsCore-6.3.1.dylib
circle: /usr/local/lib/libvtkInteractionWidgets-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersHybrid-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingGeneral-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingSources-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersModeling-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingHybrid-6.3.1.dylib
circle: /usr/local/lib/libvtkIOImage-6.3.1.dylib
circle: /usr/local/lib/libvtkDICOMParser-6.3.1.dylib
circle: /usr/local/lib/libvtkIOCore-6.3.1.dylib
circle: /usr/local/lib/libvtkmetaio-6.3.1.dylib
circle: /usr/local/lib/libvtkpng-6.3.1.dylib
circle: /usr/local/lib/libvtktiff-6.3.1.dylib
circle: /usr/local/lib/libvtkjpeg-6.3.1.dylib
circle: /usr/local/lib/libvtkInteractionStyle-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingAnnotation-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingColor-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingVolume-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingLabel-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingFreeType-6.3.1.dylib
circle: /usr/local/lib/libvtkRenderingCore-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonColor-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersExtraction-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersStatistics-6.3.1.dylib
circle: /usr/local/lib/libvtkalglib-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingFourier-6.3.1.dylib
circle: /usr/local/lib/libvtkImagingCore-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersGeometry-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersSources-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersGeneral-6.3.1.dylib
circle: /usr/local/lib/libvtkFiltersCore-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonExecutionModel-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonComputationalGeometry-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonDataModel-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonMisc-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonTransforms-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonMath-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonSystem-6.3.1.dylib
circle: /usr/local/lib/libvtkCommonCore-6.3.1.dylib
circle: /usr/local/lib/libvtksys-6.3.1.dylib
circle: /usr/local/lib/libvtkftgl-6.3.1.dylib
circle: /usr/local/lib/libvtkfreetype-6.3.1.dylib
circle: /usr/local/lib/libvtkzlib-6.3.1.dylib
circle: CMakeFiles/circle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable circle"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/circle.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/circle.dir/build: circle

.PHONY : CMakeFiles/circle.dir/build

CMakeFiles/circle.dir/requires: CMakeFiles/circle.dir/circle.cpp.o.requires

.PHONY : CMakeFiles/circle.dir/requires

CMakeFiles/circle.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/circle.dir/cmake_clean.cmake
.PHONY : CMakeFiles/circle.dir/clean

CMakeFiles/circle.dir/depend:
	cd /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5 /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5 /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build /Users/CuongPham_MBP/Documents/Study/Thesis/Read_v0.5/build/CMakeFiles/circle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/circle.dir/depend
