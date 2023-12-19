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
CMAKE_SOURCE_DIR = /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build

# Include any dependencies generated for this target.
include MathFunctions/CMakeFiles/SqrtLibrary.dir/depend.make

# Include the progress variables for this target.
include MathFunctions/CMakeFiles/SqrtLibrary.dir/progress.make

# Include the compile flags for this target's objects.
include MathFunctions/CMakeFiles/SqrtLibrary.dir/flags.make

MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o: MathFunctions/CMakeFiles/SqrtLibrary.dir/flags.make
MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o: /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2/MathFunctions/mysqrt.cxx
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o"
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o -c /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2/MathFunctions/mysqrt.cxx

MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.i"
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2/MathFunctions/mysqrt.cxx > CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.i

MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.s"
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2/MathFunctions/mysqrt.cxx -o CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.s

MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.requires:

.PHONY : MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.requires

MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.provides: MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.requires
	$(MAKE) -f MathFunctions/CMakeFiles/SqrtLibrary.dir/build.make MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.provides.build
.PHONY : MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.provides

MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.provides.build: MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o


# Object files for target SqrtLibrary
SqrtLibrary_OBJECTS = \
"CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o"

# External object files for target SqrtLibrary
SqrtLibrary_EXTERNAL_OBJECTS =

MathFunctions/libSqrtLibrary.a: MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o
MathFunctions/libSqrtLibrary.a: MathFunctions/CMakeFiles/SqrtLibrary.dir/build.make
MathFunctions/libSqrtLibrary.a: MathFunctions/CMakeFiles/SqrtLibrary.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSqrtLibrary.a"
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions && $(CMAKE_COMMAND) -P CMakeFiles/SqrtLibrary.dir/cmake_clean_target.cmake
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SqrtLibrary.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
MathFunctions/CMakeFiles/SqrtLibrary.dir/build: MathFunctions/libSqrtLibrary.a

.PHONY : MathFunctions/CMakeFiles/SqrtLibrary.dir/build

MathFunctions/CMakeFiles/SqrtLibrary.dir/requires: MathFunctions/CMakeFiles/SqrtLibrary.dir/mysqrt.cxx.o.requires

.PHONY : MathFunctions/CMakeFiles/SqrtLibrary.dir/requires

MathFunctions/CMakeFiles/SqrtLibrary.dir/clean:
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions && $(CMAKE_COMMAND) -P CMakeFiles/SqrtLibrary.dir/cmake_clean.cmake
.PHONY : MathFunctions/CMakeFiles/SqrtLibrary.dir/clean

MathFunctions/CMakeFiles/SqrtLibrary.dir/depend:
	cd /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2 /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2/MathFunctions /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions /home/renchaokang/RCK/PnC/ML_STUDY/code/cmake-3.28.0-tutorial-source/Step2_build/MathFunctions/CMakeFiles/SqrtLibrary.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : MathFunctions/CMakeFiles/SqrtLibrary.dir/depend

