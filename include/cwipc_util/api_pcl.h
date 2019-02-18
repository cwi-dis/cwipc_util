#ifndef _cwipc_util_api_pcl_h_
#define _cwipc_util_api_pcl_h_

#ifndef __cplusplus
#error "api_pcl.h requires C++"
#endif

#include "cwipc_util/api.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef  pcl::PointCloud<pcl::PointXYZRGB> cwipc_pcl_pointcloud;


_CWIPC_UTIL_EXPORT cwipc *cwipc_from_pcl(cwipc_pcl_pointcloud* pc, char **errorMessage);


#endif // _cwipc_util_api_pcl_h_