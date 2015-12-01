include_directories (${CMAKE_CURRENT_SOURCE_DIR})

if(WIN32)
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE CXX)
endif(WIN32)

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

add_definitions(-DK4W2_SRCDIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_definitions(-DK4W2_DATADIR="${CMAKE_INSTALL_PREFIX}/${PROJECT_DATA_INSTALL_DIR}")

list(APPEND SRC libk4w2.c misc.c)
list(APPEND SRC driver.c)

if(WITH_V4L2)
  add_definitions(-DWITH_V4L2)
  list(APPEND SRC driver_v4l2.c)
endif(WITH_V4L2)

if(WITH_LIBUSB)
  add_definitions(-DWITH_LIBUSB)
  include_directories(${LIBUSB_1_INCLUDE_DIRS})
  link_directories   (${LIBUSB_1_LIBRARY_DIRS})
  list(APPEND SRC driver_libusb/driver_libusb.c driver_libusb/usb_dev.c driver_libusb/usb_stream.c)
endif(WITH_LIBUSB)

if(WITH_GLEW)
  add_definitions(-DHAVE_GLEW)
  include_directories(${GLEW_INCLUDE_DIRS})
  link_directories   (${GLEW_LIBRARY_DIRS})
endif()

if(WITH_TURBOJPEG)
  add_definitions(-DHAVE_TURBOJPEG)
  include_directories(${TURBOJPEG_INCLUDE_DIRS})
  link_directories   (${TURBOJPEG_LIBRARY_DIRS})
  list(APPEND SRC decoder_cpu/color_cpu.c)
endif(WITH_TURBOJPEG)

list(APPEND SRC decoder_cpu/depth_cpu.c)

list(APPEND SRC registration.c ir_table.c)

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if (OpenCL_FOUND)
  add_definitions(-DHAVE_OPENCL)
  include_directories(${OPENCL_INCLUDE_DIRS})
  link_directories   (${OPENCL_LIBRARY_DIRS})
  list(APPEND SRC decoder_cl/depth_cl.c)
endif()

if (OpenGL_FOUND)
  include_directories(${OPENGL_INCLUDE_DIR})
  link_directories   (${OPENGL_LIBRARY_DIR})
endif()
  
if(WITH_LIBGPUJPEG)
  add_definitions    (-DHAVE_LIBGPUJPEG)
  include_directories(${LIBGPUJPEG_INCLUDE_DIRS})
  link_directories   (${LIBGPUJPEG_LIBRARY_DIRS})
  list(APPEND SRC decoder_cuda/color_cuda.c)
endif()

list(APPEND SRC decoder.c)

add_library (k4w2 SHARED ${SRC})
set_property(TARGET k4w2 PROPERTY C_STANDARD 90)

set_target_properties (k4w2 PROPERTIES
  VERSION ${PROJECT_VER}
  SOVERSION ${PROJECT_VER})

if(WITH_V4L2)
endif()
if(WITH_LIBUSB)
  target_link_libraries (k4w2 ${LIBUSB_1_LIBRARIES})
endif()
if(WITH_GLEW)
  target_link_libraries (k4w2 ${GLEW_LIBRARIES})
endif()
if(WITH_TURBOJPEG)
  target_link_libraries (k4w2 ${TURBOJPEG_LIBRARIES})
endif()
if (OPENMP_FOUND)
endif()
if(OpenCL_FOUND)
  target_link_libraries (k4w2 ${OpenCL_LIBRARIES})
endif()
if(OpenGL_FOUND)
  target_link_libraries (k4w2 ${OPENGL_LIBRARIES})
endif()
if(WITH_LIBGPUJPEG)
  target_link_libraries (k4w2 ${LIBGPUJPEG_LIBRARIES})
endif()
if(UNIX)
  target_link_libraries (k4w2 "m")
endif()

install (TARGETS k4w2
  DESTINATION "${PROJECT_LIBRARY_INSTALL_DIR}")


# Install the header files
install (FILES
  "../include/libk4w2/libk4w2.h"
  "../include/libk4w2/kinect2.h" 
  "../include/libk4w2/decoder.h"
  "../include/libk4w2/registration.h"
  DESTINATION ${PROJECT_INCLUDE_INSTALL_DIR}/${PROJECT_NAME})

#install (FILES
#  DESTINATION ${PROJECT_DATA_INSTALL_DIR})

if (OpenCL_FOUND)
  install (FILES
    "decoder_cl/depth.cl"
    DESTINATION ${PROJECT_DATA_INSTALL_DIR})
endif()

# PKGCONFIG support
IF(UNIX)
  set(PKGCONFIG_REQUIRES)
  if(WITH_LIBUSB_1)
    set(PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES} libusb-1.0")
  endif()
  if(WITH_LIBGPUJPEG)
    set(PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES} libgpujpeg")
  endif()
  configure_file (libk4w2.pc.in libk4w2.pc @ONLY)
  install (FILES "${CMAKE_CURRENT_BINARY_DIR}/libk4w2.pc"
    DESTINATION "${PROJECT_LIBRARY_INSTALL_DIR}/pkgconfig")
ENDIF()
