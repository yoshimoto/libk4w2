cmake_minimum_required (VERSION 2.6)
project (libk4w2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
include (FindOS)
include (SetupDirectories)
include (libk4w2util)
include (FindPkgConfig)
include (FindOpenMP)

find_package(OpenMP)

set (PROJECT_VER_MAJOR 0)
set (PROJECT_VER_MINOR 0)
set (PROJECT_VER_PATCH 0)
set (PROJECT_VER 
  "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

find_package(libusb-1.0 REQUIRED)

k4w2_option(WITH_V4L2   "enable v4l2 driver"       PROJECT_OS_LINUX)
k4w2_option(WITH_LIBUSB "enable libusb-1.0 driver" LIBUSB_1_FOUND)

option(BUILD_EXAMPLES "Build example programs" ON)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(DOC_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)

# Pretty much everyone is going to need the main includes
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add library project
add_subdirectory (src)

# Add examples
if(BUILD_EXAMPLES)
  add_subdirectory (examples)
endif()


