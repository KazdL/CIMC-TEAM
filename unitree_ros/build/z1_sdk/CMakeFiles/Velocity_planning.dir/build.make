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
CMAKE_SOURCE_DIR = /home/oceanyan/Files/Robotics/unitree_ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/oceanyan/Files/Robotics/unitree_ros/build

# Include any dependencies generated for this target.
include z1_sdk/CMakeFiles/Velocity_planning.dir/depend.make

# Include the progress variables for this target.
include z1_sdk/CMakeFiles/Velocity_planning.dir/progress.make

# Include the compile flags for this target's objects.
include z1_sdk/CMakeFiles/Velocity_planning.dir/flags.make

z1_sdk/CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.o: z1_sdk/CMakeFiles/Velocity_planning.dir/flags.make
z1_sdk/CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.o: /home/oceanyan/Files/Robotics/unitree_ros/src/z1_sdk/src/Velocity_planning.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/oceanyan/Files/Robotics/unitree_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object z1_sdk/CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.o"
	cd /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.o -c /home/oceanyan/Files/Robotics/unitree_ros/src/z1_sdk/src/Velocity_planning.cpp

z1_sdk/CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.i"
	cd /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/oceanyan/Files/Robotics/unitree_ros/src/z1_sdk/src/Velocity_planning.cpp > CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.i

z1_sdk/CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.s"
	cd /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/oceanyan/Files/Robotics/unitree_ros/src/z1_sdk/src/Velocity_planning.cpp -o CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.s

# Object files for target Velocity_planning
Velocity_planning_OBJECTS = \
"CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.o"

# External object files for target Velocity_planning
Velocity_planning_EXTERNAL_OBJECTS =

/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: z1_sdk/CMakeFiles/Velocity_planning.dir/src/Velocity_planning.cpp.o
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: z1_sdk/CMakeFiles/Velocity_planning.dir/build.make
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /home/oceanyan/Files/Robotics/unitree_ros/devel/lib/libik_solver.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /opt/ros/noetic/lib/libpinocchio_parsers.so.3.3.0
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/liburdfdom_sensor.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/liburdfdom_model_state.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/liburdfdom_model.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/liburdfdom_world.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /opt/ros/noetic/lib/libpinocchio_collision.so.3.3.0
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /opt/ros/noetic/lib/libpinocchio_default.so.3.3.0
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /opt/ros/noetic/lib/x86_64-linux-gnu/libhpp-fcl.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/liboctomap.so.1.9.3
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: /usr/lib/x86_64-linux-gnu/liboctomath.so.1.9.3
/home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning: z1_sdk/CMakeFiles/Velocity_planning.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/oceanyan/Files/Robotics/unitree_ros/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning"
	cd /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Velocity_planning.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
z1_sdk/CMakeFiles/Velocity_planning.dir/build: /home/oceanyan/Files/Robotics/unitree_ros/devel/lib/z1_sdk/Velocity_planning

.PHONY : z1_sdk/CMakeFiles/Velocity_planning.dir/build

z1_sdk/CMakeFiles/Velocity_planning.dir/clean:
	cd /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk && $(CMAKE_COMMAND) -P CMakeFiles/Velocity_planning.dir/cmake_clean.cmake
.PHONY : z1_sdk/CMakeFiles/Velocity_planning.dir/clean

z1_sdk/CMakeFiles/Velocity_planning.dir/depend:
	cd /home/oceanyan/Files/Robotics/unitree_ros/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/oceanyan/Files/Robotics/unitree_ros/src /home/oceanyan/Files/Robotics/unitree_ros/src/z1_sdk /home/oceanyan/Files/Robotics/unitree_ros/build /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk /home/oceanyan/Files/Robotics/unitree_ros/build/z1_sdk/CMakeFiles/Velocity_planning.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : z1_sdk/CMakeFiles/Velocity_planning.dir/depend

