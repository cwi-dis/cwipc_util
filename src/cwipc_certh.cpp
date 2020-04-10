#include <cstddef>
#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/geometry.h>


#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"


cwipc *
cwipc_from_certh(void* certhPC, uint64_t timestamp, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_from_points: incorrect apiVersion";
		}
		return NULL;
	}
    cwipc_pcl_pointcloud pclPC;
    // xxxjack
    return cwipc_from_pcl(pclPC, timestamp, errorMessage, apiVersion);
}
