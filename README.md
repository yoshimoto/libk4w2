# libk4w2
library for "Kinect for Windows v2" sensor

## Requirements

Linux
 - cmake
 - gspca/kinect2 kernel module (optional; recommended)  
   see https://github.com/yoshimoto/gspca-kinect2
 - libusb-1.0 (optional)
 - turbojpeg

Mac OSX
- cmake
- libusb-1.0 
- turbojpeg

Windows (not tested yet because I have no windows env.)
 - cmake
 - libusb-1.0
 - turbojpeg

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
