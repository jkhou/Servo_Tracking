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
CMAKE_SOURCE_DIR = /home/hjk/catkin_ws/src/servo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hjk/catkin_ws/src/servo

# Include any dependencies generated for this target.
include CMakeFiles/SCServo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/SCServo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SCServo.dir/flags.make

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o: include/SCServo_Linux/SCS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hjk/catkin_ws/src/servo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o -c /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCS.cpp

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCS.cpp > CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.i

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCS.cpp -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.s

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.requires:

.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.requires

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.provides: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.requires
	$(MAKE) -f CMakeFiles/SCServo.dir/build.make CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.provides.build
.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.provides

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.provides.build: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o


CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o: include/SCServo_Linux/SCSCL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hjk/catkin_ws/src/servo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o -c /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCSCL.cpp

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCSCL.cpp > CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.i

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCSCL.cpp -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.s

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.requires:

.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.requires

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.provides: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.requires
	$(MAKE) -f CMakeFiles/SCServo.dir/build.make CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.provides.build
.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.provides

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.provides.build: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o


CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o: include/SCServo_Linux/SCSerial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hjk/catkin_ws/src/servo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o -c /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCSerial.cpp

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCSerial.cpp > CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.i

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SCSerial.cpp -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.s

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.requires:

.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.requires

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.provides: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.requires
	$(MAKE) -f CMakeFiles/SCServo.dir/build.make CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.provides.build
.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.provides

CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.provides.build: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o


CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o: include/SCServo_Linux/SMSBL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hjk/catkin_ws/src/servo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o -c /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SMSBL.cpp

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SMSBL.cpp > CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.i

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SMSBL.cpp -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.s

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.requires:

.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.requires

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.provides: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.requires
	$(MAKE) -f CMakeFiles/SCServo.dir/build.make CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.provides.build
.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.provides

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.provides.build: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o


CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o: CMakeFiles/SCServo.dir/flags.make
CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o: include/SCServo_Linux/SMSCL.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hjk/catkin_ws/src/servo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o -c /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SMSCL.cpp

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SMSCL.cpp > CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.i

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hjk/catkin_ws/src/servo/include/SCServo_Linux/SMSCL.cpp -o CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.s

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.requires:

.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.requires

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.provides: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.requires
	$(MAKE) -f CMakeFiles/SCServo.dir/build.make CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.provides.build
.PHONY : CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.provides

CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.provides.build: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o


# Object files for target SCServo
SCServo_OBJECTS = \
"CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o" \
"CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o" \
"CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o" \
"CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o" \
"CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o"

# External object files for target SCServo
SCServo_EXTERNAL_OBJECTS =

devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o
devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o
devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o
devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o
devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o
devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/build.make
devel/lib/libSCServo.so: CMakeFiles/SCServo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hjk/catkin_ws/src/servo/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library devel/lib/libSCServo.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SCServo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SCServo.dir/build: devel/lib/libSCServo.so

.PHONY : CMakeFiles/SCServo.dir/build

CMakeFiles/SCServo.dir/requires: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCS.cpp.o.requires
CMakeFiles/SCServo.dir/requires: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSCL.cpp.o.requires
CMakeFiles/SCServo.dir/requires: CMakeFiles/SCServo.dir/include/SCServo_Linux/SCSerial.cpp.o.requires
CMakeFiles/SCServo.dir/requires: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSBL.cpp.o.requires
CMakeFiles/SCServo.dir/requires: CMakeFiles/SCServo.dir/include/SCServo_Linux/SMSCL.cpp.o.requires

.PHONY : CMakeFiles/SCServo.dir/requires

CMakeFiles/SCServo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SCServo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SCServo.dir/clean

CMakeFiles/SCServo.dir/depend:
	cd /home/hjk/catkin_ws/src/servo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hjk/catkin_ws/src/servo /home/hjk/catkin_ws/src/servo /home/hjk/catkin_ws/src/servo /home/hjk/catkin_ws/src/servo /home/hjk/catkin_ws/src/servo/CMakeFiles/SCServo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/SCServo.dir/depend
