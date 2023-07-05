#include <chrono>
#include <thread>
#include <inttypes.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_util/internal.h"


cwipc_tiledsource *
cwipc_capturer(const char *configFilename, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_capturer: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }
		return NULL;
	}
    char* msgbuf = (char*)malloc(1024);
    snprintf(msgbuf, 1024, "cwipc_capturer: not yet implemented");
    *errorMessage = msgbuf;
	return NULL;
}

int _cwipc_register_capturer(const char *name, bool isCamera, _cwipc_functype_count_devices *countFunc, _cwipc_func_capturer_factory *factoryFunc) {
    fprintf(stderr, "xxxjack _cwipc_register_capturer(%s, ...)\n", name);
    return 0;
}
