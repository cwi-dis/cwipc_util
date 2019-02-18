#ifndef _cwipc_util_api_h_
#define _cwipc_util_api_h_

#include <stdint.h>

// For Windows ensure that the symbols are imported from a DLL, unless we're compiling the DLL itself.
#ifndef _CWIPC_UTIL_EXPORT
#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllimport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#endif

struct cwipc_point {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

#ifdef __cplusplus

#ifndef _CWIPC_PCL_POINTCLOUD_DEFINED
typedef void* cwipc_pcl_pointcloud;
#define _CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED
#endif //_CWIPC_PCL_POINTCLOUD_DEFINED

class cwipc {
public:
    virtual ~cwipc() {};
    virtual void free() = 0;
    virtual uint32_t timestamp() = 0;
    virtual size_t get_uncompressed_size() = 0;
    virtual int copy_uncompressed(struct cwipc_point *, size_t size) = 0;
    virtual cwipc_pcl_pointcloud access_pcl_pointcloud() = 0;
};
#else
typedef struct _cwipc {
} cwipc;
typedef struct _cwipc_pcl_pointcloud {
} *cwipc_pcl_pointcloud;
#endif

#ifdef __cplusplus
extern "C" {
#endif

_CWIPC_UTIL_EXPORT cwipc *cwipc_read(const char *filename, char **errorMessage);
_CWIPC_UTIL_EXPORT int cwipc_write(const char *filename, cwipc *pointcloud, char **errorMessage);

_CWIPC_UTIL_EXPORT cwipc *cwipc_read_debugdump(const char *filename, char **errorMessage);
_CWIPC_UTIL_EXPORT int cwipc_write_debugdump(const char *filename, cwipc *pointcloud, char **errorMessage);

_CWIPC_UTIL_EXPORT void cwipc_free(cwipc *pc);
_CWIPC_UTIL_EXPORT uint32_t cwipc_timestamp(cwipc *pc);
_CWIPC_UTIL_EXPORT size_t cwipc_get_uncompressed_size(cwipc *pc);
_CWIPC_UTIL_EXPORT int cwipc_copy_uncompressed(cwipc *pc, struct cwipc_point *, size_t size);
_CWIPC_UTIL_EXPORT cwipc *cwipc_from_points(struct cwipc_point* points, size_t size, int npoint, char **errorMessage);

#ifdef __cplusplus
}
#endif

#endif // _cwipc_util_api_h_
