# CWI Pointcloud utility library

This repository is a utility library for using `cwipc` objects to represent pointclouds. It is part of the `cwipc` suite, <https://github.com/cwi-dis/cwipc>, and should generally be installed or built as part of that suite. 

## General design considerations

The idea behind a `cwipc` pointcloud object is that it represents a pointcloud (with x/y/z coordinates, r/g/b values and possibly information on which camera angle the point was captured from) without having to include the whole [PCL Point Cloud Library](https://github.com/PointCloudLibrary/pcl) API if you do not need it.

the idea is that you can pass a `cwipc` object around without knowing what is inside it, and if the receiving end also has PCL knowledge it can access the pointcloud data without it having been copied. And if a receiver *does* need access to the individual point data it will only be copied when it arrives there (so a receiver can allocate buffers in the right type of memory, etc).

The current design document is in [pointcloud-api.md](pointcloud-api.md). 

The library contains utility functions to read and write a `cwipc` object from a `.ply` file. There are also functions to read/write the binary representation of a `cwipc` but these are mainly for debugging (the binary representation is not portable between machines).

Bindings for C, C++ and Python are included. C# bindings are not currently included, ask us.



## Installing

For use within VRtogether you can get pre-built zipfiles (or tgzfiles for Mac/Linux) from <https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/releases>. Download the most recent release with a normal v_X_._Y_._Z_ name.

[![pipeline status](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/badges/master/pipeline.svg)](https://baltig.viaccess-orca.com:8443/VRT/nativeclient-group/cwipc_util/commits/master)

### Windows

- Install PCL 1.11 via <https://pointclouds.org/downloads>. Make sure you select the "add to %PATH% for all users" option.
- Install Python 3.7 into `C:\Python37` _for all users_ and add to PATH _for all users_.
	- There are issues with DLL search paths on Windows in Python 3.8 and newer. So for now use 3.7.
- Create a folder where you will install _all_ VRtogether DLLs and EXEs, for example `C:\vrtogether\installed`.
- Extract the `cwipc_util_win1064_vX.Y.zip` file into `c:\vrtogether`. This will create `bin`, `lib` and `include` folders inside the `C:\vrtogether\installed` folder.
- Add the `c:\vrtogether\installed\bin` folder to the `%PATH%` system environment variable.

### OSX

> **Note** As of this writing (March 2021) when you use an M1 (Apple Silicon) you must build everything using Rosetta x86 compatibility mode, because not all dependencies have been ported to M1 yet. Installing an x86 `brew` and ensuring it is in `$PATH` before an M1 `brew` should work.

- Install _brew_, and then 

  ```
  brew install pcl glfw3
  ```
  
- You may have to force Python 3.8 (as opposed to 3.9) because open3d isn't up to date yet. If you have to do this, use the following:

  ```
  brew install python@3.8
  # Check where it is installed, use that below in cmake:
  cmake -DPython3_EXECUTABLE=/usr/local/Cellar/python@3.8/3.8.8_1/bin/python3.8 ..
  ```
  
- Extract the gzip file into `/usr/local`:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  ```
  

### Ubuntu 20.04

- Install _PCL_ with `apt-get install libpcl-dev`.
- Extract the gzip file into `/usr/local`:

  ```
  cd /usr/local
  [sudo] tar xfv .../cwipc_util_osx1012_vX.Y.tgz
  ```

## Building from source

If you want to build _cwipc\_util_ from source follow the next steps:

### OSX

- Install XCode
- Install various dependencies:

  ```
  brew install cmake
  brew install pcl
  ```
  
- Building with Makefiles:

  ```
  mkdir build
  cd build
  cmake ..
  make
  make test # optional
  [sudo] make install
  ```
  
- Note: whether or not you need `sudo` depends on your setup. First try without and if it fails try with `sudo`.

### Linux

Install dependencies:

```
apt-get install cmake
apt-get install libpcl
```

Build:

  ```
  mkdir build
  cd build
  cmake ..
  make
  make test # optional
  sudo make install
  ```

### Windows


- Install Windows 10-64
- Install Visual Studio Community Edition
- Install Notepad++
- Install Git (checkout as-is, commit as-is for Unix newlines)
- Install CMake
- Install PCL 1.8 via <https://pointclouds.org/downloads>
	- Ensure all needed DLL directories are added to `%PATH%`. Especially the `OpenNI` directory seems to be forgotten by the PCL installer.
- Configure and build. The following set of commands is known to work (in bash):

  ```
  instdir=`pwd`/../installed
  mkdir -p build
  cd build
  cmake .. -G "Visual Studio 16 2019" -DCMAKE_INSTALL_PREFIX="$instdir" 
  cmake --build . --config Release
  cmake --build . --config Release --target RUN_TESTS
  cmake --build . --config Release --target INSTALL

  ```
- Add the installation bin directory (`.../DIR/installed` from the examples above) to your system-wide environment variable `PATH`. This will allow dependent packages to find all the DLLs and such.
 