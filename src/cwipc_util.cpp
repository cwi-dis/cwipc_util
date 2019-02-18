#include <cstddef>

#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"

cwipc *
cwipc_read(const char *filename, char **errorMessage)
{
	if (errorMessage) *errorMessage = (char *)"Not yet implemented";
	return NULL;
}

int 
cwipc_write(const char *filename, cwipc *pointcloud, char **errorMessage)
{
	if (errorMessage) *errorMessage = (char *)"Not yet implemented";
	return -1;
}

cwipc *
cwipc_read_debugdump(const char *filename, char **errorMessage)
{
	if (errorMessage) *errorMessage = (char *)"Not yet implemented";
	return NULL;
}

int 
cwipc_write_debugdump(const char *filename, cwipc *pointcloud, char **errorMessage)
{
	if (errorMessage) *errorMessage = (char *)"Not yet implemented";
	return -1;
}

cwipc *
cwipc_from_pcl(cwipc_pcl_pointcloud* pc, char **errorMessage)
{
	if (errorMessage) *errorMessage = (char *)"Not yet implemented";
	return NULL;
}