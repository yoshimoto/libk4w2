# libk4w2
library for "Kinect for Windows v2" sensor

## Requirements

Linux
 - cmake
 - gspca/kinect2 kernel module (optional; recommended) 
   - See https://github.com/yoshimoto/gspca-kinect2
 - libusb-1.0 (optional)
 - OpenCL (optional)
   - required for optimized depth decording
 - nvJPEG (optional; recommended)
 - GPUJPEG (optional)
   - A modified version of GPUJPEG is required. 
   It is available at https://sourceforge.net/u/yosimoto/gpujpeg/ci/master/tree/
 - turbojpeg (optional)
 - OpenMP (optional)
   - Some codes will be optimized when OpenMP is available
 - OpenCV (optional)
   - OpenCV-based sample codes are available in directory "examples/"
 - GLEW (optional)
   - Requred for OpenGL interoperability; see examples/opengl.c for details.

Mac OSX
 - cmake
 - libusb-1.0 
 - turbojpeg (optional)
 - OpenCL (optional)
 - GPUJPEG (optional, not tested yet)
 - OpenMP (optional)
 - OpenCV (optional)
 - GLEW (optional)

Windows (not tested yet because I have no windows env.)
 - cmake
 - libusb-1.0
 - turbojpeg
 - OpenCL (optional)
 - GPUJPEG (optional)
 - OpenMP (optional)
 - OpenCV (optional)
 - GLEW (optional)

## Build & install

```
$ mkdir build
$ cd
$ cmake ..
$ make
```

## Run test codes

```
$ ./bin/simple
```

```
$ ./bin/opengl
$ ./bin/opencv
$ ./bin/liveview
```


If you want to specify header/library's path, you can use CMAKE_INCLUDE_PATH and CMAKE_LIBRARY_PATH as follows;
```
$ cmake .. -DCMAKE_INCLUDE_PATH=/path/to/your/include-dir -DCMAKE_LIBRARY_PATH=/path/to/your/lib-dir
```
## Sample codes

Some sample codes are available in the examples/ directory.
- examples/simple.c
  - Simple template that just grabs raw images.
- examples/opengl.c
  - Display live video by using OpenGL.
  - This code can work quite efficiently by using OpenGL-CUDA and OpenGL-OpenCL interoperabilities.
- examples/opencv.c
  - Display live video by using OpenCV.
- examples/liveview.cpp
  - Demonstration of depth-color image registration

To build these examples, use Makefile.* in examples/.
```
$ make
$ make -f Makefile.opengl
```

# Aknowledgements
This library is based on the following discussions and source codes;
- libfreenect2, https://github.com/OpenKinect/libfreenect2
- Analyzing Kinect 2 ISO data, https://groups.google.com/forum/#!topic/openkinect/8Aa8ab3aAs4

Special thanks to the people in the OpenKinect project!!!

Hiromasa YOSHIMOTO
