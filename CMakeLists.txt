cmake_minimum_required (VERSION 2.6)
project (libk4w2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/)
include (FindOS)
include (SetupDirectories)
include (libk4w2util)
include (FindPkgConfig)
include (FindOpenMP)
include (FindOpenCL)
include (CheckFunctionExists)


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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")

# Library version
#
set (PROJECT_VER_MAJOR 0)
set (PROJECT_VER_MINOR 0)
set (PROJECT_VER_PATCH 0)
set (PROJECT_VER 
  "${PROJECT_VER_MAJOR}.${PROJECT_VER_MINOR}.${PROJECT_VER_PATCH}")

find_package(libusb-1.0)
find_package(OpenGL)
find_package(GLEW)

pkg_check_modules(GLFW3 glfw3)
pkg_check_modules(LIBGPUJPEG libgpujpeg)

k4w2_option(WITH_V4L2       "enable v4l2 driver"       ON IF PROJECT_OS_LINUX)
k4w2_option(WITH_LIBUSB     "enable libusb-1.0 driver" ON IF LIBUSB_1_FOUND)
k4w2_option(WITH_LIBGPUJPEG "enable libgpujpeg decoder" ON IF LIBGPUJPEG_FOUND)
k4w2_option(WITH_GLEW       "enable opengl interoperability" ON IF GLEW_FOUND)
k4w2_option(WITH_GLFW3      "Build glfw3-based example " ON IF GLFW3_FOUND)
k4w2_option(BUILD_EXAMPLES  "Build example programs" ON)



# check gpujpeg version
if(WITH_LIBGPUJPEG)
  cmake_push_check_state(RESET)
  set(CMAKE_REQUIRED_FLAGS "${LIBGPUJPEG_CFLAGS} ${LIBGPUJPEG_LDFLAGS}")
  set(CMAKE_REQUIRED_INCLUDES "${LIBGPUJPEG_INCLUDES}")
  set(CMAKE_REQUIRED_LIBRARIES "${LIBGPUJPEG_LIBRARIES}")
  CHECK_FUNCTION_EXISTS(gpujpeg_decoder_fetch HAVE_GPUJPEG_DECODER_FETCH)
  if(NOT HAVE_GPUJPEG_DECODER_FETCH)
    message(FATAL_ERROR "\n** ERROR **\nlibk4w2 needs modified version of GPUJPEG, which is available at git://git.code.sf.net/u/yosimoto/gpujpeg.\nSee https://github.com/yoshimoto/libk4w2/issues/3 for details.\n")
  endif(NOT HAVE_GPUJPEG_DECODER_FETCH)
  cmake_pop_check_state()
endif(WITH_LIBGPUJPEG)


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
status("       v4l2 driver : " WITH_V4L2        THEN Yes     ELSE No)
status(" libusb-1.0 driver : " WITH_LIBUSB      THEN Yes     ELSE No)
status("")
status("            OpenCV : " OPENCV_FOUND     THEN Yes     ELSE No)
status("            OpenMP : " OPENMP_FOUND     THEN Yes     ELSE No)
status("            OpenCL : " OpenCL_FOUND     THEN Yes     ELSE No)
status("           GPUJPEG : " WITH_LIBGPUJPEG  THEN Yes     ELSE No)
status("            OpenGL : " OpenGL_FOUND     THEN Yes     ELSE No)
status("              GLEW : " WITH_GLEW        THEN Yes     ELSE No)
status("             GLFW3 : " WITH_GLFW3       THEN Yes     ELSE No)
status("")
status("      Install path : ${CMAKE_INSTALL_PREFIX}")
status("")
status(" ***** ")

