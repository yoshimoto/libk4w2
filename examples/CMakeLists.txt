
pkg_check_modules(OPENCV opencv)
if(OPENCV_FOUND)
  include_directories(${OPENCV_INCLUDE_DIRS})
  add_executable(liveview liveview.cpp)
  target_link_libraries (liveview k4w2 ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES})
  add_executable(opencv opencv.cpp)
  target_link_libraries (opencv k4w2 ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES})
endif(OPENCV_FOUND)

if (LIBGPUJPEG_FOUND)
  link_directories(${LIBGPUJPEG_LIBRARY_DIRS})
endif()

add_executable(simple simple.c)
target_link_libraries (simple k4w2)
target_link_libraries (simple ${LIBGPUJPEG_LIBRARIES})

if (WITH_GLFW3)
add_executable(opengl opengl.c)
target_link_libraries (opengl k4w2)
target_link_libraries (opengl ${GLFW3_LIBRARIES} ${GLEW_LIBRARIES} ${OPENGL_LIBRARIES})
endif()