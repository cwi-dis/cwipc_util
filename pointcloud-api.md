# CWI Pointcloud object

The CWI pointcloud object is intended as an abstract object representing a pointcloud.

It is used by the following libraries:

- [CWIPC Util](https://github.com/cwi-dis/cwipc_util), this library.
- [CWI Codec](https://github.com/cwi-dis/cwi_codec_lib)
- [Realsense Capturer](https://github.com/cwi-dis/VRTogether-capture)

The library can be used from C and C++. In the latter case it will expose a virtual object API, in the former case an opaque object is passsed as the first argument to many functions.

In case the library is used from C++ it can also export a PCL (Point Cloud Library) API, which allows access to the underlying PCL implementations of the pointclouds.

## cwipc C++ interface

Here are the C++ methods (with all the `virtual` and `= 0;` and such removed for readability):

```
class cwipc {
	void free();
	uint32_t timestamp();
	size_t get_uncompressed_size();
	void copy_uncompressed(struct pointcloud *, size_t size);
	pcl_pointcloud *access_pcl_pointcloud();
};
```

## cwipc C interface

tbd

## Utility function interface

tbd

## Debug/test function interface

tbd

## realsense2 interface

tbd

## codec interface

tbd

## Historic reasons for design of API for pointclouds
We need APIs for capturing compressing and decompressing pointclouds, and while we're at it we might as well add APIs for reading and writing them to files.

There are two libraries involved:

- [CWI Codec](https://github.com/cwi-dis/cwi_codec_lib)
- [Realsense Capturer](https://github.com/cwi-dis/VRTogether-capture)

There are currently two main users of the API:

- pcl2dash (which isn't in github at the moment)
- Unity Renderer (in i2cat repo, to be provided)

The API needs to be accessible from C++ (for pcl2dash) and C# (for Unity). For the latter we need a C API that is bridgeable to C#, i.e. the _unmanaged pointers_ (C# terminology) need to be convertible to managed data structures with as little copying as possible. It would be good if the C API has such a form that

-  it would also be usable from C directly (not only C#), 
-  it would be usable on other platforms than Windows,
-  it would be usable in other languages (Python comes to mind).

The pointclouds are stored internally (in the codec and capturer) as [libpcl](http://pointclouds.org) data structures. I think these are in turn based on [Boost](https://www.boost.org) data structures.

It is a requirement that the API can be accessed trough a dynamic library, possibly loaded at runtime (requirement for C#).

It is a requirement that the DLL or program that has allocated storage is also responsible for deallocating it (to forestall C++ runtime system issues on Windows).

It is a requirement that the PCL datastructures (uncompressed point clouds) can be transferred between the capturer and the encoder without copying.

It is a requirement that the uncompressed pointclouds can also be made available in a non-PCL non-boost format (for the Unity renderer).

It is a requirement that _pcl2dash_ does not have to include PCL or Boost or other headers (it only has to transfer the pointcloud data structures between the capturer and the codec).

## Header files, stubs

There are a number of distinct header files for different types of use:

- a combined C/C++ header file `cwipc.h` which does not require including PCL or Boost headers, and which allows copying the pointcloud data (see below). The header uses the usual tricks for hiding the C++ code from C and adding the `extern "C"` bits where needed. 
- another header file `cwipc_ex.h` (or `.hpp`?) which does include the PCL and Boost headers and is used inside the capture and codec libraries (and possibly in other programs that need to access the PCL representations). This file includes `cwipc.h`.
- Separate header files for the capturer and the codec APIs.

> Discussion point: It feels elegant to have to C# bridge (the code that currently lives in Unity, in <https://github.com/cwi-dis/VRTogether-PointCloud-Rendering/tree/master/Unity/Assets/Scripts/CWI>) be part of our API. But I don't know how well that integrates with Unity (can unity refer to scripts that don't live inside their directory structure? Or should we simply say that the scripts need to be copied verbatim into the directory structure, just as the DLLs are also copied at the moment?).

## Data structures

Uncompressed pointclouds produced by the capturer or decoder are represented by an opaque datastructure `opaque_pointcloud`. In C++ this is a class in the `cwipc` namespace with methods to access it. In C it is a `struct cwipc_opaque_pointcloud` that is passed as the first parameter to the accessor functions.

Here are the C++ methods (with all the `virtual` and `= 0;` and such removed for readability):

```
namespace cwipc {
struct pointcloud;
class pcl_pointcloud;

class opaque_pointcloud {
	void free();
	uint32_t timestamp();
	size_t get_uncompressed_size();
	void copy_uncompressed(struct pointcloud *, size_t size);
	pcl_pointcloud *access_pcl_pointcloud();
};
};
```

> Discussion point: what is the correct type for the timestamp?

> Discussion point: if a pointcloud has been captured by multiple cameras and we need the multiple angles sometimes separately, sometimes together, is this a different datastructure (`opaque_multi_pointcloud`) with different accessors, or do we add `partial` variants to the get/copy/access methods to allow getting only partial pointclouds?
 
Note that the opaque pointcloud object is the owner of the the underlying PCL pointcloud, and calling its `free()` method will invalidate the PCL pointcloud reference returned by `access_pcl_pointcloud()`. In case a consumer of the pointcloud wants to access the individual points it first gets the size in bytes required (with `get_uncompressed_size()`, then allocates a buffer of that size, then passes that buffer to `copy_uncompressed()`. It is suggested that the main program (i.e. the caller of the capturer or decoder method that returned this `opaque_pointcloud`) is responsible of calling free (so not having the compressor call free all by itself after being done with the data).

In case the C api is used the caller has to ensure that it calls the `cwipc_free()` function from the same DLL that originally allocated the opaque pointcloud.

Here is the C API:

```
struct cwipc_opaque_pointcloud;
struct cwipc_pointcloud;

void cwipc_free(struct cwipc_opaque_pointcloud *);
uint32_t cwipc_timestamp(struct cwipc_opaque_pointcloud *);
size_t cwipc_get_uncompressed_size(struct cwipc_opaque_pointcloud *);
void cwipc_copy_uncompressed(struct cwipc_opaque_pointcloud *, struct pointcloud *, size_t size);

```

Uncompressed pointclouds usable by consumers are represented as simple structs:

```
struct cwpipc_point {
	int32_t x, y, z;
	uint8_t r, g, b;
};

struct cwipc_pointcloud {
	uint32_t npoints;
	struct cwipc_point points[1];
};
```

> Discussion point: should a point contain an indication of which cameras contributed to it? 

> Discussion point: what are the types for x/y/z and r/g/b?

> Discussion point: are compressed pointclouds represented simply as a byte string (i.e. a `void *` and a `size_t`)? Or should we do a similar thing which `opaque_compressed_pointcloud` to ensure alloc/free is handled by the correct library? Or should we do the `std::stringstream` thing currently used, even though that means there's an issue with C use (as opposed to C++) and we have yet another memory copy...

> To do: the exact structure of the `cwipc` namespace and the `cwpic_` prefix for C use needs to be worked out.

## Capturer API

TBD

## Compressor API

TBD

## Decompressor API

TBD

## File reader API

```
cwipc_pointcloud *cwipc_from_file(char *filename);
```