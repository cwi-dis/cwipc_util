# CWI Pointcloud utility library

This repository builds a utility library for using `cwipc` objects to represent pointclouds.

The idea behind a `cwipc` pointcloud object is that it represents a pointcloud (with x/y/z coordinates, r/g/b values and possibly information on which camera angle the point was captured from) without having to include the whole [PCL Point Cloud Library](https://github.com/PointCloudLibrary/pcl) API if you do not need it.

the idea is that you can pass a `cwipc` object around without knowing what is inside it, and if the receiving end also has PCL knowledge it can access the pointcloud data without it having been copied. And if a receiver *does* need access to the individual point data it will only be copied when it arrives there (so a receiver can allocate buffers in the right type of memory, etc).

The current design document is in [pointcloud-api.md](pointcloud-api.md). 

The library contains utility functions to read and write a `cwipc` object from a `.ply` file. There are also functions to read/write the binary representation of a `cwipc` but these are mainly for debugging (the binary representation is not portable between machines).

## Building

### OSX

tbd

### Linux

tbd

### Windows

tbd