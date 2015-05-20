# - Try to find turbojpeg
# Once done this will define
#
#  TURBOJPEG_FOUND - system has turbojpeg
#  TURBOJPEG_INCLUDE_DIRS - the turbojpeg include directory
#  TURBOJPEG_LIBRARIES - Link these to use turbojpeg
#  TURBOJPEG_DEFINITIONS - Compiler switches required for using turbojpeg
#
#  Adapted from cmake-modules Google Code project
#
#  Copyright (c) 2015 Andreas Schneider <mail@cynapses.org>
#
#  (Changes for turbojpeg) Copyright (c) 2015 Hiromasa YOSHIMOTO <hrmsysmt@gmail.com>
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

if (TURBOJPEG_LIBRARIES AND TURBOJPEG_INCLUDE_DIRS)
  # in cache already
  set(TURBOJPEG_FOUND TRUE)
else (TURBOJPEG_LIBRARIES AND TURBOJPEG_INCLUDE_DIRS)
  find_path(TURBOJPEG_INCLUDE_DIR
    NAMES
	turbojpeg.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
	PATH_SUFFIXES
	  turbojpeg
  )

  find_library(TURBOJPEG_LIBRARY
    NAMES
      turbojpeg
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
  )

  set(TURBOJPEG_INCLUDE_DIRS
    ${TURBOJPEG_INCLUDE_DIR}
  )
  set(TURBOJPEG_LIBRARIES
    ${TURBOJPEG_LIBRARY}
)

  if (TURBOJPEG_INCLUDE_DIRS AND TURBOJPEG_LIBRARIES)
     set(TURBOJPEG_FOUND TRUE)
  endif (TURBOJPEG_INCLUDE_DIRS AND TURBOJPEG_LIBRARIES)

  if (TURBOJPEG_FOUND)
    if (NOT turbojpeg_FIND_QUIETLY)
      message(STATUS "Found turbojpeg:")
	  message(STATUS " - Includes: ${TURBOJPEG_INCLUDE_DIRS}")
	  message(STATUS " - Libraries: ${TURBOJPEG_LIBRARIES}")
    endif (NOT turbojpeg_FIND_QUIETLY)
  else (TURBOJPEG_FOUND)
    if (turbojpeg_FIND_REQUIRED)
      message(FATAL_ERROR "Could not find turbojpeg")
    endif (turbojpeg_FIND_REQUIRED)
  endif (TURBOJPEG_FOUND)

  # show the TURBOJPEG_INCLUDE_DIRS and TURBOJPEG_LIBRARIES variables only in the advanced view
  mark_as_advanced(TURBOJPEG_INCLUDE_DIRS TURBOJPEG_LIBRARIES)

endif (TURBOJPEG_LIBRARIES AND TURBOJPEG_INCLUDE_DIRS)