# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/jiyguo/clion-2021.3.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/jiyguo/clion-2021.3.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jiyguo/ur_ws/src/dmp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jiyguo/ur_ws/src/dmp/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/dmp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/dmp.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/dmp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dmp.dir/flags.make

CMakeFiles/dmp.dir/src/dmp.cpp.o: CMakeFiles/dmp.dir/flags.make
CMakeFiles/dmp.dir/src/dmp.cpp.o: ../src/dmp.cpp
CMakeFiles/dmp.dir/src/dmp.cpp.o: CMakeFiles/dmp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiyguo/ur_ws/src/dmp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dmp.dir/src/dmp.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/dmp.dir/src/dmp.cpp.o -MF CMakeFiles/dmp.dir/src/dmp.cpp.o.d -o CMakeFiles/dmp.dir/src/dmp.cpp.o -c /home/jiyguo/ur_ws/src/dmp/src/dmp.cpp

CMakeFiles/dmp.dir/src/dmp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp.dir/src/dmp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiyguo/ur_ws/src/dmp/src/dmp.cpp > CMakeFiles/dmp.dir/src/dmp.cpp.i

CMakeFiles/dmp.dir/src/dmp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp.dir/src/dmp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiyguo/ur_ws/src/dmp/src/dmp.cpp -o CMakeFiles/dmp.dir/src/dmp.cpp.s

CMakeFiles/dmp.dir/src/fourier_approx.cpp.o: CMakeFiles/dmp.dir/flags.make
CMakeFiles/dmp.dir/src/fourier_approx.cpp.o: ../src/fourier_approx.cpp
CMakeFiles/dmp.dir/src/fourier_approx.cpp.o: CMakeFiles/dmp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiyguo/ur_ws/src/dmp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/dmp.dir/src/fourier_approx.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/dmp.dir/src/fourier_approx.cpp.o -MF CMakeFiles/dmp.dir/src/fourier_approx.cpp.o.d -o CMakeFiles/dmp.dir/src/fourier_approx.cpp.o -c /home/jiyguo/ur_ws/src/dmp/src/fourier_approx.cpp

CMakeFiles/dmp.dir/src/fourier_approx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp.dir/src/fourier_approx.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiyguo/ur_ws/src/dmp/src/fourier_approx.cpp > CMakeFiles/dmp.dir/src/fourier_approx.cpp.i

CMakeFiles/dmp.dir/src/fourier_approx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp.dir/src/fourier_approx.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiyguo/ur_ws/src/dmp/src/fourier_approx.cpp -o CMakeFiles/dmp.dir/src/fourier_approx.cpp.s

CMakeFiles/dmp.dir/src/radial_approx.cpp.o: CMakeFiles/dmp.dir/flags.make
CMakeFiles/dmp.dir/src/radial_approx.cpp.o: ../src/radial_approx.cpp
CMakeFiles/dmp.dir/src/radial_approx.cpp.o: CMakeFiles/dmp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiyguo/ur_ws/src/dmp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/dmp.dir/src/radial_approx.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/dmp.dir/src/radial_approx.cpp.o -MF CMakeFiles/dmp.dir/src/radial_approx.cpp.o.d -o CMakeFiles/dmp.dir/src/radial_approx.cpp.o -c /home/jiyguo/ur_ws/src/dmp/src/radial_approx.cpp

CMakeFiles/dmp.dir/src/radial_approx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp.dir/src/radial_approx.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiyguo/ur_ws/src/dmp/src/radial_approx.cpp > CMakeFiles/dmp.dir/src/radial_approx.cpp.i

CMakeFiles/dmp.dir/src/radial_approx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp.dir/src/radial_approx.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiyguo/ur_ws/src/dmp/src/radial_approx.cpp -o CMakeFiles/dmp.dir/src/radial_approx.cpp.s

CMakeFiles/dmp.dir/src/linear_approx.cpp.o: CMakeFiles/dmp.dir/flags.make
CMakeFiles/dmp.dir/src/linear_approx.cpp.o: ../src/linear_approx.cpp
CMakeFiles/dmp.dir/src/linear_approx.cpp.o: CMakeFiles/dmp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jiyguo/ur_ws/src/dmp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/dmp.dir/src/linear_approx.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/dmp.dir/src/linear_approx.cpp.o -MF CMakeFiles/dmp.dir/src/linear_approx.cpp.o.d -o CMakeFiles/dmp.dir/src/linear_approx.cpp.o -c /home/jiyguo/ur_ws/src/dmp/src/linear_approx.cpp

CMakeFiles/dmp.dir/src/linear_approx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dmp.dir/src/linear_approx.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jiyguo/ur_ws/src/dmp/src/linear_approx.cpp > CMakeFiles/dmp.dir/src/linear_approx.cpp.i

CMakeFiles/dmp.dir/src/linear_approx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dmp.dir/src/linear_approx.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jiyguo/ur_ws/src/dmp/src/linear_approx.cpp -o CMakeFiles/dmp.dir/src/linear_approx.cpp.s

# Object files for target dmp
dmp_OBJECTS = \
"CMakeFiles/dmp.dir/src/dmp.cpp.o" \
"CMakeFiles/dmp.dir/src/fourier_approx.cpp.o" \
"CMakeFiles/dmp.dir/src/radial_approx.cpp.o" \
"CMakeFiles/dmp.dir/src/linear_approx.cpp.o"

# External object files for target dmp
dmp_EXTERNAL_OBJECTS =

devel/lib/libdmp.so: CMakeFiles/dmp.dir/src/dmp.cpp.o
devel/lib/libdmp.so: CMakeFiles/dmp.dir/src/fourier_approx.cpp.o
devel/lib/libdmp.so: CMakeFiles/dmp.dir/src/radial_approx.cpp.o
devel/lib/libdmp.so: CMakeFiles/dmp.dir/src/linear_approx.cpp.o
devel/lib/libdmp.so: CMakeFiles/dmp.dir/build.make
devel/lib/libdmp.so: CMakeFiles/dmp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jiyguo/ur_ws/src/dmp/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX shared library devel/lib/libdmp.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dmp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/dmp.dir/build: devel/lib/libdmp.so
.PHONY : CMakeFiles/dmp.dir/build

CMakeFiles/dmp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dmp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dmp.dir/clean

CMakeFiles/dmp.dir/depend:
	cd /home/jiyguo/ur_ws/src/dmp/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jiyguo/ur_ws/src/dmp /home/jiyguo/ur_ws/src/dmp /home/jiyguo/ur_ws/src/dmp/cmake-build-debug /home/jiyguo/ur_ws/src/dmp/cmake-build-debug /home/jiyguo/ur_ws/src/dmp/cmake-build-debug/CMakeFiles/dmp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dmp.dir/depend

