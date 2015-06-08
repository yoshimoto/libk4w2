# libk4w2
library for "Kinect for Windows v2" sensor

## Requirements

Linux
 - cmake
 - gspca/kinect2 kernel module (optional; recommended) 
  * See https://github.com/yoshimoto/gspca-kinect2
 - libusb-1.0 (optional)
 - turbojpeg (optional)
 - GPUJPEG (optional)
  * A modified version of GPUJPEG is required. 
   It is available at https://sourceforge.net/u/yosimoto/gpujpeg/ci/master/tree/
 - OpenMP (optional)
  * Some codes will be optimized when OpenMP is available
 - OpenCV (optional)
  * OpenCV-based sample codes are available in directory "examples/"

Mac OSX
 - cmake
 - libusb-1.0 
 - turbojpeg (optional)
 - GPUJPEG (optional, not tested yet)
 - OpenMP (optional)
 - OpenCV (optional)

Windows (not tested yet because I have no windows env.)
 - cmake
 - libusb-1.0
 - turbojpeg
 - GPUJPEG (optional)
 - OpenMP (optional)
 - OpenCV (optional)

## Build & install

```
$ mkdir build
$ cd
$ cmake ..
$ make
```

```
$ ./bin/liveview
```


If you want to specify header/library's path, you can use CMAKE_INCLUDE_PATH and CMAKE_LIBRARY_PATH as follows;
```
$ cmake .. -DCMAKE_INCLUDE_PATH=/path/to/your/include-dir -DCMAKE_LIBRARY_PATH=/path/to/your/lib-dir
```


# Aknowledgements
This library is based on the following discussions and source codes;
- libfreenect2, https://github.com/OpenKinect/libfreenect2
- Analyzing Kinect 2 ISO data, https://groups.google.com/forum/#!topic/openkinect/8Aa8ab3aAs4

Special thanks to the people in the OpenKinect project!!!

Hiromasa YOSHIMOTO
