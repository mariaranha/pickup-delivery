# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.12.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.12.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1

# Include any dependencies generated for this target.
include test/CMakeFiles/euler_test.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/euler_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/euler_test.dir/flags.make

test/CMakeFiles/euler_test.dir/euler_test.cc.o: test/CMakeFiles/euler_test.dir/flags.make
test/CMakeFiles/euler_test.dir/euler_test.cc.o: test/euler_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/euler_test.dir/euler_test.cc.o"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/euler_test.dir/euler_test.cc.o -c /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test/euler_test.cc

test/CMakeFiles/euler_test.dir/euler_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/euler_test.dir/euler_test.cc.i"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test/euler_test.cc > CMakeFiles/euler_test.dir/euler_test.cc.i

test/CMakeFiles/euler_test.dir/euler_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/euler_test.dir/euler_test.cc.s"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test/euler_test.cc -o CMakeFiles/euler_test.dir/euler_test.cc.s

# Object files for target euler_test
euler_test_OBJECTS = \
"CMakeFiles/euler_test.dir/euler_test.cc.o"

# External object files for target euler_test
euler_test_EXTERNAL_OBJECTS =

test/euler_test: test/CMakeFiles/euler_test.dir/euler_test.cc.o
test/euler_test: test/CMakeFiles/euler_test.dir/build.make
test/euler_test: lemon/libemon.a
test/euler_test: /usr/local/lib/libglpk.dylib
test/euler_test: test/CMakeFiles/euler_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable euler_test"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/euler_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/euler_test.dir/build: test/euler_test

.PHONY : test/CMakeFiles/euler_test.dir/build

test/CMakeFiles/euler_test.dir/clean:
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test && $(CMAKE_COMMAND) -P CMakeFiles/euler_test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/euler_test.dir/clean

test/CMakeFiles/euler_test.dir/depend:
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1 /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1 /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/test/CMakeFiles/euler_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/euler_test.dir/depend

