# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_COMMAND = "/Applications/CMake 2.8-12.app/Contents/bin/cmake"

# The command to remove a file.
RM = "/Applications/CMake 2.8-12.app/Contents/bin/cmake" -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = "/Applications/CMake 2.8-12.app/Contents/bin/ccmake"

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build

# Include any dependencies generated for this target.
include CMakeFiles/ProjectiveTextureMapping.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ProjectiveTextureMapping.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ProjectiveTextureMapping.dir/flags.make

CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/main.cpp

CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/main.cpp > CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/main.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o

CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/mesh.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/mesh.cpp

CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/mesh.cpp > CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/mesh.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o

CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/texture.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/texture.cpp

CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/texture.cpp > CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/texture.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o

CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/controls.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/controls.cpp

CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/controls.cpp > CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/controls.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o

CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/loadShader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/loadShader.cpp

CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/loadShader.cpp > CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/loadShader.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o

CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/objloader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/objloader.cpp

CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/objloader.cpp > CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/objloader.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o

CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o: CMakeFiles/ProjectiveTextureMapping.dir/flags.make
CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o: /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/textureLoader.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o -c /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/textureLoader.cpp

CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/textureLoader.cpp > CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.i

CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping/common/textureLoader.cpp -o CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.s

CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.requires:
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.requires

CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.provides: CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.requires
	$(MAKE) -f CMakeFiles/ProjectiveTextureMapping.dir/build.make CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.provides.build
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.provides

CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.provides.build: CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o

# Object files for target ProjectiveTextureMapping
ProjectiveTextureMapping_OBJECTS = \
"CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o" \
"CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o" \
"CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o" \
"CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o" \
"CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o" \
"CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o" \
"CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o"

# External object files for target ProjectiveTextureMapping
ProjectiveTextureMapping_EXTERNAL_OBJECTS =

ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/build.make
ProjectiveTextureMapping: /usr/lib/libGLEW.dylib
ProjectiveTextureMapping: /opt/local/lib/libglfw.dylib
ProjectiveTextureMapping: /opt/local/lib/libassimp.dylib
ProjectiveTextureMapping: /opt/local/lib/libMagick++-6.Q16.dylib
ProjectiveTextureMapping: CMakeFiles/ProjectiveTextureMapping.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ProjectiveTextureMapping"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ProjectiveTextureMapping.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ProjectiveTextureMapping.dir/build: ProjectiveTextureMapping
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/build

CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/main.cpp.o.requires
CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/mesh.cpp.o.requires
CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/texture.cpp.o.requires
CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/common/controls.cpp.o.requires
CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/common/loadShader.cpp.o.requires
CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/common/objloader.cpp.o.requires
CMakeFiles/ProjectiveTextureMapping.dir/requires: CMakeFiles/ProjectiveTextureMapping.dir/common/textureLoader.cpp.o.requires
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/requires

CMakeFiles/ProjectiveTextureMapping.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ProjectiveTextureMapping.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/clean

CMakeFiles/ProjectiveTextureMapping.dir/depend:
	cd /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build /Users/avrdan/Work/DIKU/Thesis/Code/ProjectiveTextureMapping-build/CMakeFiles/ProjectiveTextureMapping.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ProjectiveTextureMapping.dir/depend

