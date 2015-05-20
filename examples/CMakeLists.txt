
pkg_check_modules(OPENCV opencv)
if(OPENCV_FOUND)
  include_directories(${OPENCV_INCLUDE_DIRS})
  add_executable(liveview liveview.cpp)
  target_link_libraries (liveview k4w2 ${OPENCV_LDFLAGS} ${OPENCV_LIBRARIES})
endif(OPENCV_FOUND)

add_executable(simple simple.c)
target_link_libraries (simple k4w2)
