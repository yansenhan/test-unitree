# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build

# Include any dependencies generated for this target.
include CMakeFiles/example_velocity.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example_velocity.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_velocity.dir/flags.make

CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.o: CMakeFiles/example_velocity.dir/flags.make
CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.o: ../examples/example_velocity.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.o"
	/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.o -c /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/examples/example_velocity.cpp

CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.i"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/examples/example_velocity.cpp > CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.i

CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.s"
	/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/examples/example_velocity.cpp -o CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.s

# Object files for target example_velocity
example_velocity_OBJECTS = \
"CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.o"

# External object files for target example_velocity
example_velocity_EXTERNAL_OBJECTS =

example_velocity: CMakeFiles/example_velocity.dir/examples/example_velocity.cpp.o
example_velocity: CMakeFiles/example_velocity.dir/build.make
example_velocity: CMakeFiles/example_velocity.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_velocity"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_velocity.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_velocity.dir/build: example_velocity

.PHONY : CMakeFiles/example_velocity.dir/build

CMakeFiles/example_velocity.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_velocity.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_velocity.dir/clean

CMakeFiles/example_velocity.dir/depend:
	cd /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2 /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2 /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build /home/hanyansen/hanyansen/unitree_robot/test-unitree/unitree_legged_sdk-3.2/build/CMakeFiles/example_velocity.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example_velocity.dir/depend

