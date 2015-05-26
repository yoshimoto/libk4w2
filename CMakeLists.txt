cmake_minimum_required (VERSION 2.6)
project (libk4w2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
include (FindOS)
include (SetupDirectories)
include (libk4w2util)
include (FindPkgConfig)
include (FindOpenMP)


#
# Platform specific configrations
#
if(APPLE)
    set(CMAKE_MACOSX_RPATH ON)
    set(CMAKE_SKIP_BUILD_RPATH FALSE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
        set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    endif()
endif()

if(WIN32)
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE CXX)
endif(WIN32)


#
# Compiler options
#
find_package(OpenMP)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")

# Libary version
#
set (PROJECT_VER_MAJOR 0)
set (PROJECT_VER_MINOR 0)
set (PROJECT_VER_PATCH 0)
set (PROJECT_VER 
  "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

find_package(libusb-1.0)

k4w2_option(WITH_V4L2      "enable v4l2 driver"       ON IF PROJECT_OS_LINUX)
k4w2_option(WITH_LIBUSB    "enable libusb-1.0 driver" ON IF LIBUSB_1_FOUND)
k4w2_option(BUILD_EXAMPLES "Build example programs" ON)

# Target directories
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(DOC_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)
include_directories (${CMAKE_CURRENT_SOURCE_DIR}/include)

# Add library project
#
add_subdirectory (src)

# Add examples
#
if(BUILD_EXAMPLES)
  add_subdirectory (examples)
endif()

# Show configration 
#
status(" ***** ")
status("")
status("       v4l2 driver : " WITH_V4L2     THEN On ELSE Off)
status(" libusb-1.0 driver : " WITH_LIBUSB   THEN On ELSE Off)
status("")
status("            OpenMP : " OPENMP_FOUND     THEN Yes     ELSE No)
status("            OpenCV : " OPENCV_FOUND     THEN Yes     ELSE No)
status("")
status("      Install path : ${CMAKE_INSTALL_PREFIX}")
status("")
status(" ***** ")

