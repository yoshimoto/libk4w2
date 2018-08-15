# - Try to find nvJPEG
# Once done this will define
#
#  NVJPEG_FOUND - system has nvjpeg
#  NVJPEG_INCLUDE_DIRS - the nvjpeg include directory
#  NVJPEG_LIBRARIES - Link these to use nvjpeg
#  NVJPEG_DEFINITIONS - Compiler switches required for using nvjpeg
#
#  Adapted from cmake-modules Google Code project
#
#  Copyright (c) 2015 Andreas Schneider <mail@cynapses.org>
#
#  (Changes for nvjpeg) Copyright (c) 2015 Hiromasa YOSHIMOTO <hrmsysmt@gmail.com>
#
# Redistribution and use is allowed according to the terms of the New BSD license.
#
# CMake-Modules Project New BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the name of the CMake-Modules Project nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

if (NVJPEG_LIBRARIES AND NVJPEG_INCLUDE_DIRS)
  # in cache already
  set(NVJPEG_FOUND TRUE)
else (NVJPEG_LIBRARIES AND NVJPEG_INCLUDE_DIRS)
  find_path(NVJPEG_INCLUDE_DIR
    NAMES
	nvjpeg.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
      ${CMAKE_SOURCE_DIR}/cuda-linux64-nvjpeg/include
    PATH_SUFFIXES
	nvjpeg
  )

  find_library(NVJPEG_LIBRARY
    NAMES
      nvjpeg
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      ${CMAKE_SOURCE_DIR}/cuda-linux64-nvjpeg/lib64
  )

  set(NVJPEG_INCLUDE_DIRS
    ${NVJPEG_INCLUDE_DIR}
  )
  set(NVJPEG_LIBRARIES
    ${NVJPEG_LIBRARY}
)

  if (NVJPEG_INCLUDE_DIRS AND NVJPEG_LIBRARIES)
     set(NVJPEG_FOUND TRUE)
  endif (NVJPEG_INCLUDE_DIRS AND NVJPEG_LIBRARIES)

  if (NVJPEG_FOUND)
    if (NOT NVJPEG_FIND_QUIETLY)
      message(STATUS "Found nvjpeg:")
	  message(STATUS " - Includes: ${NVJPEG_INCLUDE_DIRS}")
	  message(STATUS " - Libraries: ${NVJPEG_LIBRARIES}")
    endif (NOT NVJPEG_FIND_QUIETLY)
  else (NVJPEG_FOUND)
    if (NVJPEG_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find nvjpeg")
    endif (NVJPEG_FIND_REQUIRED)
  endif (NVJPEG_FOUND)

  # show the NVJPEG_INCLUDE_DIRS and NVJPEG_LIBRARIES variables only in the advanced view
  mark_as_advanced(NVJPEG_INCLUDE_DIRS NVJPEG_LIBRARIES)

endif (NVJPEG_LIBRARIES AND NVJPEG_INCLUDE_DIRS)