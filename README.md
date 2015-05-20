# libk4w2
library for Kinect for Windows v2


## requirements

* linux
- cmake
- gspca/kinect2 kernel module (optional; recommended)
- libusb-1.0 (optional)
- turbojpeg

* Mac OSX
- cmake
- libusb-1.0 
- turbojpeg

* Windows (not tested yet)
- cmake
- libusb-1.0
- turbojpeg

## build & install

```
$ mkdir build
$ cd
$ cmake ..
```

```
$ ./bin/liveview
```

```
$ cmake .. -DCMAKE_INCLUDE_PATH=/path/to/include -DCMAKE_LIBRARY_PATH=/path/to/lib
```
