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
include demo/CMakeFiles/lgf_demo.dir/depend.make

# Include the progress variables for this target.
include demo/CMakeFiles/lgf_demo.dir/progress.make

# Include the compile flags for this target's objects.
include demo/CMakeFiles/lgf_demo.dir/flags.make

demo/CMakeFiles/lgf_demo.dir/lgf_demo.cc.o: demo/CMakeFiles/lgf_demo.dir/flags.make
demo/CMakeFiles/lgf_demo.dir/lgf_demo.cc.o: demo/lgf_demo.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object demo/CMakeFiles/lgf_demo.dir/lgf_demo.cc.o"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo && /Library/Developer/CommandLineTools/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lgf_demo.dir/lgf_demo.cc.o -c /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo/lgf_demo.cc

demo/CMakeFiles/lgf_demo.dir/lgf_demo.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lgf_demo.dir/lgf_demo.cc.i"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo/lgf_demo.cc > CMakeFiles/lgf_demo.dir/lgf_demo.cc.i

demo/CMakeFiles/lgf_demo.dir/lgf_demo.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lgf_demo.dir/lgf_demo.cc.s"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo/lgf_demo.cc -o CMakeFiles/lgf_demo.dir/lgf_demo.cc.s

# Object files for target lgf_demo
lgf_demo_OBJECTS = \
"CMakeFiles/lgf_demo.dir/lgf_demo.cc.o"

# External object files for target lgf_demo
lgf_demo_EXTERNAL_OBJECTS =

demo/lgf_demo: demo/CMakeFiles/lgf_demo.dir/lgf_demo.cc.o
demo/lgf_demo: demo/CMakeFiles/lgf_demo.dir/build.make
demo/lgf_demo: lemon/libemon.a
demo/lgf_demo: /usr/local/lib/libglpk.dylib
demo/lgf_demo: demo/CMakeFiles/lgf_demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable lgf_demo"
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lgf_demo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demo/CMakeFiles/lgf_demo.dir/build: demo/lgf_demo

.PHONY : demo/CMakeFiles/lgf_demo.dir/build

demo/CMakeFiles/lgf_demo.dir/clean:
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo && $(CMAKE_COMMAND) -P CMakeFiles/lgf_demo.dir/cmake_clean.cmake
.PHONY : demo/CMakeFiles/lgf_demo.dir/clean

demo/CMakeFiles/lgf_demo.dir/depend:
	cd /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1 /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1 /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo /Users/fkm/Dropbox/tiziu/programming/branch_and_cut_and_connectivity/connectivity/lab_mc658/lemon/lemon-1.3.1/demo/CMakeFiles/lgf_demo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : demo/CMakeFiles/lgf_demo.dir/depend

