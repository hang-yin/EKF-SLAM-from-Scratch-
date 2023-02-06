# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(turtlelib)

include_directories(include/turtlelib)

set(HEADER_FILES include/turtlelib/diff_drive.hpp include/turtlelib/rigid2d.hpp)

# Find your dependencies.
# Many libraries ship with files that allow CMake to find them
# Then general behavior is to call "find_package" but the options
# provided are package specific.  Usually there is then a CMAKE variable
# That is defined to reference the library
# here: we find the eigen library as per the instruction
# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
# find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Create an executable from the following source code files
# The Name of the executable creates a cmake "target"
add_executable(frame_main src/frame_main.cpp)
target_link_libraries(frame_main turtlelib)

# Create a library.  Can specify if it is shared or static but usually
# you don't need or want to.
# name is the name of the library without the extension or lib prefix
# name creates a cmake "target"
add_library(turtlelib src/diff_drive.cpp src/rigid2d.cpp ${HEADER_FILES})

# Use target_link_libraries to add dependencies to a "target"
# (e.g., a library or executable)
# This will automatically add all required library files
# that need to be linked
# and paths to th locations of header files
# target_link_libraries(Name Eigen3::Eigen)

# Use target_include_directories so that #include"mylibrary/header.hpp" works
# The use of the <BUILD_INTERFACE> and <INSTALL_INTERFACE> is because when
# Using the library from the build directory or after installation
# During build, the headers are read from the source code directory
# When used from the installed location, headers are in the 
# system include/ directory
target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for a target
target_compile_options(turtlelib PUBLIC -Wall)

install (TARGETS turtlelib EXPORT turtlelib-targets )

install(EXPORT turtlelib-targets
        FILE turtlelib-config.cmake
        NAMESPACE turtlelib::
        DESTINATION lib/cmake/turtlelib)
