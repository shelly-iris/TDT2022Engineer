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
CMAKE_SOURCE_DIR = /home/shelly/gitee/look/TDT2022Engineer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shelly/gitee/look/TDT2022Engineer/build

# Include any dependencies generated for this target.
include CMakeFiles/TDTvision2022_engineer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/TDTvision2022_engineer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/TDTvision2022_engineer.dir/flags.make

CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o: CMakeFiles/TDTvision2022_engineer.dir/flags.make
CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shelly/gitee/look/TDT2022Engineer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o -c /home/shelly/gitee/look/TDT2022Engineer/main.cpp

CMakeFiles/TDTvision2022_engineer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TDTvision2022_engineer.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shelly/gitee/look/TDT2022Engineer/main.cpp > CMakeFiles/TDTvision2022_engineer.dir/main.cpp.i

CMakeFiles/TDTvision2022_engineer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TDTvision2022_engineer.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shelly/gitee/look/TDT2022Engineer/main.cpp -o CMakeFiles/TDTvision2022_engineer.dir/main.cpp.s

CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.requires

CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.provides: CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/TDTvision2022_engineer.dir/build.make CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.provides

CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.provides.build: CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o


CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o: CMakeFiles/TDTvision2022_engineer.dir/flags.make
CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o: ../ksconnect/ksconnect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shelly/gitee/look/TDT2022Engineer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o -c /home/shelly/gitee/look/TDT2022Engineer/ksconnect/ksconnect.cpp

CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shelly/gitee/look/TDT2022Engineer/ksconnect/ksconnect.cpp > CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.i

CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shelly/gitee/look/TDT2022Engineer/ksconnect/ksconnect.cpp -o CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.s

CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.requires:

.PHONY : CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.requires

CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.provides: CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.requires
	$(MAKE) -f CMakeFiles/TDTvision2022_engineer.dir/build.make CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.provides.build
.PHONY : CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.provides

CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.provides.build: CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o


CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o: CMakeFiles/TDTvision2022_engineer.dir/flags.make
CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o: ../usart/usart.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shelly/gitee/look/TDT2022Engineer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o -c /home/shelly/gitee/look/TDT2022Engineer/usart/usart.cpp

CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shelly/gitee/look/TDT2022Engineer/usart/usart.cpp > CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.i

CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shelly/gitee/look/TDT2022Engineer/usart/usart.cpp -o CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.s

CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.requires:

.PHONY : CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.requires

CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.provides: CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.requires
	$(MAKE) -f CMakeFiles/TDTvision2022_engineer.dir/build.make CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.provides.build
.PHONY : CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.provides

CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.provides.build: CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o


CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o: CMakeFiles/TDTvision2022_engineer.dir/flags.make
CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o: ../tool/tool.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shelly/gitee/look/TDT2022Engineer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o -c /home/shelly/gitee/look/TDT2022Engineer/tool/tool.cpp

CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shelly/gitee/look/TDT2022Engineer/tool/tool.cpp > CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.i

CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shelly/gitee/look/TDT2022Engineer/tool/tool.cpp -o CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.s

CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.requires:

.PHONY : CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.requires

CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.provides: CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.requires
	$(MAKE) -f CMakeFiles/TDTvision2022_engineer.dir/build.make CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.provides.build
.PHONY : CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.provides

CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.provides.build: CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o


# Object files for target TDTvision2022_engineer
TDTvision2022_engineer_OBJECTS = \
"CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o" \
"CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o" \
"CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o" \
"CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o"

# External object files for target TDTvision2022_engineer
TDTvision2022_engineer_EXTERNAL_OBJECTS =

bin/TDTvision2022_engineer: CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o
bin/TDTvision2022_engineer: CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o
bin/TDTvision2022_engineer: CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o
bin/TDTvision2022_engineer: CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o
bin/TDTvision2022_engineer: CMakeFiles/TDTvision2022_engineer.dir/build.make
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_img_hash.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_sfm.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: /usr/local/lib/libopencv_world.so.4.4.0
bin/TDTvision2022_engineer: CMakeFiles/TDTvision2022_engineer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shelly/gitee/look/TDT2022Engineer/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable bin/TDTvision2022_engineer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TDTvision2022_engineer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/TDTvision2022_engineer.dir/build: bin/TDTvision2022_engineer

.PHONY : CMakeFiles/TDTvision2022_engineer.dir/build

CMakeFiles/TDTvision2022_engineer.dir/requires: CMakeFiles/TDTvision2022_engineer.dir/main.cpp.o.requires
CMakeFiles/TDTvision2022_engineer.dir/requires: CMakeFiles/TDTvision2022_engineer.dir/ksconnect/ksconnect.cpp.o.requires
CMakeFiles/TDTvision2022_engineer.dir/requires: CMakeFiles/TDTvision2022_engineer.dir/usart/usart.cpp.o.requires
CMakeFiles/TDTvision2022_engineer.dir/requires: CMakeFiles/TDTvision2022_engineer.dir/tool/tool.cpp.o.requires

.PHONY : CMakeFiles/TDTvision2022_engineer.dir/requires

CMakeFiles/TDTvision2022_engineer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/TDTvision2022_engineer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/TDTvision2022_engineer.dir/clean

CMakeFiles/TDTvision2022_engineer.dir/depend:
	cd /home/shelly/gitee/look/TDT2022Engineer/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shelly/gitee/look/TDT2022Engineer /home/shelly/gitee/look/TDT2022Engineer /home/shelly/gitee/look/TDT2022Engineer/build /home/shelly/gitee/look/TDT2022Engineer/build /home/shelly/gitee/look/TDT2022Engineer/build/CMakeFiles/TDTvision2022_engineer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/TDTvision2022_engineer.dir/depend

