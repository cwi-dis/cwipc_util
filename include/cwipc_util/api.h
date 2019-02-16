#ifndef _cwipc_util_api_h_
#define _cwipc_util_api_h_
// For Windows ensure that the symbols are imported from a DLL, unless we're compiling the DLL itself.
#ifndef _CWIPC_UTIL_EXPORT
#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllimport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#endif

typedef struct _cwipc {
} cwipc;

#ifdef __cplusplus
extern "C" {
#endif

_CWIPC_UTIL_EXPORT cwipc *cwipc_read(const char *filename, char **errorMessage);
_CWIPC_UTIL_EXPORT int cwipc_write(const char *filename, cwipc *pointcloud, char **errorMessage);

_CWIPC_UTIL_EXPORT cwipc *cwipc_read_debugdump(const char *filename, char **errorMessage);
_CWIPC_UTIL_EXPORT int cwipc_write_debugdump(const char *filename, cwipc *pointcloud, char **errorMessage);

#ifdef __cplusplus
}
#endif

#endif // _cwipc_util_api_h_