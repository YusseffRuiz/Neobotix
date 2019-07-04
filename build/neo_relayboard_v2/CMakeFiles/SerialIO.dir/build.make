# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/yusseff/Neobotix/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yusseff/Neobotix/build

# Include any dependencies generated for this target.
include neo_relayboard_v2/CMakeFiles/SerialIO.dir/depend.make

# Include the progress variables for this target.
include neo_relayboard_v2/CMakeFiles/SerialIO.dir/progress.make

# Include the compile flags for this target's objects.
include neo_relayboard_v2/CMakeFiles/SerialIO.dir/flags.make

neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o: neo_relayboard_v2/CMakeFiles/SerialIO.dir/flags.make
neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o: /home/yusseff/Neobotix/src/neo_relayboard_v2/common/src/SerialIO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yusseff/Neobotix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o"
	cd /home/yusseff/Neobotix/build/neo_relayboard_v2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o -c /home/yusseff/Neobotix/src/neo_relayboard_v2/common/src/SerialIO.cpp

neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.i"
	cd /home/yusseff/Neobotix/build/neo_relayboard_v2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yusseff/Neobotix/src/neo_relayboard_v2/common/src/SerialIO.cpp > CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.i

neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.s"
	cd /home/yusseff/Neobotix/build/neo_relayboard_v2 && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yusseff/Neobotix/src/neo_relayboard_v2/common/src/SerialIO.cpp -o CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.s

neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.requires:

.PHONY : neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.requires

neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.provides: neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.requires
	$(MAKE) -f neo_relayboard_v2/CMakeFiles/SerialIO.dir/build.make neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.provides.build
.PHONY : neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.provides

neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.provides.build: neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o


# Object files for target SerialIO
SerialIO_OBJECTS = \
"CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o"

# External object files for target SerialIO
SerialIO_EXTERNAL_OBJECTS =

/home/yusseff/Neobotix/devel/lib/libSerialIO.so: neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o
/home/yusseff/Neobotix/devel/lib/libSerialIO.so: neo_relayboard_v2/CMakeFiles/SerialIO.dir/build.make
/home/yusseff/Neobotix/devel/lib/libSerialIO.so: neo_relayboard_v2/CMakeFiles/SerialIO.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yusseff/Neobotix/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/yusseff/Neobotix/devel/lib/libSerialIO.so"
	cd /home/yusseff/Neobotix/build/neo_relayboard_v2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialIO.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
neo_relayboard_v2/CMakeFiles/SerialIO.dir/build: /home/yusseff/Neobotix/devel/lib/libSerialIO.so

.PHONY : neo_relayboard_v2/CMakeFiles/SerialIO.dir/build

neo_relayboard_v2/CMakeFiles/SerialIO.dir/requires: neo_relayboard_v2/CMakeFiles/SerialIO.dir/common/src/SerialIO.cpp.o.requires

.PHONY : neo_relayboard_v2/CMakeFiles/SerialIO.dir/requires

neo_relayboard_v2/CMakeFiles/SerialIO.dir/clean:
	cd /home/yusseff/Neobotix/build/neo_relayboard_v2 && $(CMAKE_COMMAND) -P CMakeFiles/SerialIO.dir/cmake_clean.cmake
.PHONY : neo_relayboard_v2/CMakeFiles/SerialIO.dir/clean

neo_relayboard_v2/CMakeFiles/SerialIO.dir/depend:
	cd /home/yusseff/Neobotix/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yusseff/Neobotix/src /home/yusseff/Neobotix/src/neo_relayboard_v2 /home/yusseff/Neobotix/build /home/yusseff/Neobotix/build/neo_relayboard_v2 /home/yusseff/Neobotix/build/neo_relayboard_v2/CMakeFiles/SerialIO.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : neo_relayboard_v2/CMakeFiles/SerialIO.dir/depend

