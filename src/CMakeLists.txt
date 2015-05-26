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

list(APPEND SRC libk4w2.c misc.c)

list(APPEND SRC driver.c)

if(WITH_V4L2)
  list(APPEND SRC driver_v4l2.c)
  add_definitions(-DWITH_V4L2)
endif(WITH_V4L2)

if(WITH_LIBUSB)
  include_directories(${LIBUSB_1_INCLUDE_DIRS})
  list(APPEND SRC driver_libusb/driver_libusb.c driver_libusb/usb_dev.c driver_libusb/usb_stream.c)
  add_definitions(-DWITH_LIBUSB)
endif(WITH_LIBUSB)

list(APPEND SRC decoder.c)
find_package(TurboJPEG)
if(TURBOJPEG_FOUND)
  include_directories(${TURBOJPEG_INCLUDE_DIRS})
  add_definitions(-DHAVE_TURBOJPEG)
else(TURBOJPEG_FOUND)
  message(FATAL_ERROR "TurboJPEG is missing, aborting!")
endif(TURBOJPEG_FOUND)
add_definitions(-DK4W2_DATADIR="${CMAKE_CURRENT_SOURCE_DIR}")
list(APPEND SRC decoder_color_cpu.c decoder_depth_cpu.c)

list(APPEND SRC registration.c)

if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_library (k4w2 SHARED ${SRC})
set_target_properties (k4w2 PROPERTIES
  VERSION ${PROJECT_VER}
  SOVERSION ${PROJECT_VER})
if(WITH_LIBUSB)
  target_link_libraries (k4w2 ${LIBUSB_1_LIBRARIES})
endif()
if(TURBOJPEG_FOUND)
  target_link_libraries (k4w2 ${TURBOJPEG_LIBRARIES})
endif()
target_link_libraries (k4w2 "m")

install (TARGETS k4w2
  DESTINATION "${PROJECT_LIBRARY_INSTALL_DIR}")


# Install the header files
install (FILES
  "../include/libk4w2/libk4w2.h"
  "../include/libk4w2/kinect2.h" 
  "../include/libk4w2/decoder.h"
  "../include/libk4w2/registration.h"
  DESTINATION ${PROJECT_INCLUDE_INSTALL_DIR})


# PKGCONFIG support
IF(UNIX AND NOT APPLE)
  if(LIBUSB_1_FOUND)
    set(PKGCONFIG_REQUIRES)
    list(APPEND PKGCONFIG_REQUIRES libusb-1.0)
  endif()
  configure_file (libk4w2.pc.in libk4w2.pc @ONLY)
  install (FILES "${CMAKE_CURRENT_BINARY_DIR}/libk4w2.pc"
    DESTINATION "${PROJECT_LIBRARY_INSTALL_DIR}/pkgconfig")
ENDIF()
