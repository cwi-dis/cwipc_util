# CWI Pointcloud utility library

This repository builds a utility library for using `cwipc` objects to represent pointclouds.

The idea behind a `cwipc` pointcloud object is that it represents a pointcloud (with x/y/z coordinates, r/g/b values and possibly information on which camera angle the point was captured from) without having to include the whole [PCL Point Cloud Library](https://github.com/PointCloudLibrary/pcl) API if you do not need it.

the idea is that you can pass a `cwipc` object around without knowing what is inside it, and if the receiving end also has PCL knowledge it can access the pointcloud data without it having been copied. And if a receiver *does* need access to the individual point data it will only be copied when it arrives there (so a receiver can allocate buffers in the right type of memory, etc).

The current design document is in [pointcloud-api.md](pointcloud-api.md). 

The library contains utility functions to read and write a `cwipc` object from a `.ply` file. There are also functions to read/write the binary representation of a `cwipc` but these are mainly for debugging (the binary representation is not portable between machines).

## Installing

For use within VRtogether you can get pre-built zipfiles (or tgzfiles for Mac/Linux) from <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/releases>. Download the most recent release with a normal v_X_._Y_._Z_ name.

### Windows

- Install PCL 1.8 from <https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.8.1/PCL-1.8.1-AllInOne-msvc2017-win64.exe>. Make sure you select the "add to %PATH% for all users" option.
- Create a folder where you will install _all_ VRtogether DLLs and EXEs, for example `C:\vrtogether`.
- Extract the `cwipc_util_win1064_vX.Y.zip` file into this folder. This will create `bin`, `lib` and `include` folders inside the `C:\vrtogether` folder.
- Add the `c:\vrtogether\bin` folder to the `%PATH%` system environment variable.

### OSX

- Install _brew_, and then `brew install pcl`.
- Extract the gzip file in the root directory, `/`. This will put the actual contents into `/usr/local`:

  ```
  cd /
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  ```
  
### Ubuntu 18.04

- Install _PCL_ with `apt-get install libpcl-dev`.
- Extract the gzip file in the root directory, `/`. This will put the actual contents into `/usr/local`:

  ```
  cd /
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  ```

## Building

If you want to build _cwipc\_util_ from source follow the next steps:

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
- *Note*: for reasons unknown the file `CMakeFiles/cwipc_util-config.cmake` tends to disappear, this will have `make install` fail. Fix by running the following command:
  ```
  git checkout -- CMakeFiles/cwipc_util-config.cmake
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
	- Set the installation directory. Suggested is `.../DIR/installed` where `DIR` is the directory where you have cloned all the repos.
	- Generate VS projects in a `build` subdirectory.
	- Alternatively, if you use *git bash*, use the following commands:

	```
	mkdir build
	cd build
	cmake .. -G "Visual Studio 15 Win64"
	```
- Open Visual Studio solution, build *ALL_BUILD*, then build *RUN_TESTS* then build *INSTALL*.
- Add the installation bin directory (`.../DIR/installed` from the examples above) to your system-wide environment variable `PATH`. This will allow dependent packages to find all the DLLs and such.
- *Note*: for reasons unknown the file `CMakeFiles/cwipc_util-config.cmake` tends to disappear, this will have `make install` fail. Fix by running the following command:
  ```
  git checkout -- CMakeFiles/cwipc_util-config.cmake
  ```
 