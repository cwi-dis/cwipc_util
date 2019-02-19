# CWI Pointcloud utility library

This repository builds a utility library for using `cwipc` objects to represent pointclouds.

The idea behind a `cwipc` pointcloud object is that it represents a pointcloud (with x/y/z coordinates, r/g/b values and possibly information on which camera angle the point was captured from) without having to include the whole [PCL Point Cloud Library](https://github.com/PointCloudLibrary/pcl) API if you do not need it.

the idea is that you can pass a `cwipc` object around without knowing what is inside it, and if the receiving end also has PCL knowledge it can access the pointcloud data without it having been copied. And if a receiver *does* need access to the individual point data it will only be copied when it arrives there (so a receiver can allocate buffers in the right type of memory, etc).

The current design document is in [pointcloud-api.md](pointcloud-api.md). 

The library contains utility functions to read and write a `cwipc` object from a `.ply` file. There are also functions to read/write the binary representation of a `cwipc` but these are mainly for debugging (the binary representation is not portable between machines).

## Building

### OSX

- Install XCode
- Install various dependencies:

  ```
  brew install cmake
  brew install pcl
  ```
- Building with XCode:

  ```
  mkdir build-xcode
  cd build-code
  cmake .. -G Xcode
  open cwipc_util.xcodeproj
  ```
  
  and then build it.
  
- Building with Makefiles:

  ```
  mkdir build-makefile
  cd build-makefile
  cmake ..
  make
  make test # optional
  make install # optional
  ```

### Linux

tbd

### Windows


- Install Windows 10-64
- Install Visual Studio Community Edition
- Install Notepad++
- Install Git (checkout as-is, commit as-is for Unix newlines)
- Install CMake
- Install PCL 1.8 from <https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win64.exe>
- Configure the repository using cmake-gui.
	- Make sure you select the right Visual Studio version and 64bits.
	- Generate VS projects in a `build` subdirectory.
	- Alternatively, if you use *git bash*, use the following commands:

	```
	mkdir build
	cd build
	cmake .. -G "Visual Studio 15 Win64"
	```
- Open Visual Studio solution, build.
- To make things run I had to add the following directories to system environment variable PATH:
	- tbd
 