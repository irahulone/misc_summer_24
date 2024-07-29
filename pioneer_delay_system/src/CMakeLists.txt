cmake_minimum_required(VERSION 3.10)

# Set the project name
project(DelayCLI)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Include OpenCV directories
include_directories(${OpenCV_INCLUDE_DIRS})

# Add the executable
add_executable(DelayCLI delay_cli.cpp)

# Include directories
target_include_directories(DelayCLI PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link OpenCV and pthread libraries
target_link_libraries(DelayCLI ${OpenCV_LIBS} pthread)

# Optionally specify header files to be part of the project (not strictly necessary for compilation)


