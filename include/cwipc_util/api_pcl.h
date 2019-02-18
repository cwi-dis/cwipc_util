#ifndef _cwipc_util_api_pcl_h_
#define _cwipc_util_api_pcl_h_

#ifndef __cplusplus
#error "api_pcl.h requires C++"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef _CWIPC_PCL_POINTCLOUD_DEFINED
typedef  pcl::PointCloud<pcl::PointXYZRGB> cwipc_pcl_pointcloud;
#define _CWIPC_PCL_POINTCLOUD_DEFINED
#endif //_CWIPC_PCL_POINTCLOUD_DEFINED

#ifdef _CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED
#warning cwipc_pcl_pointcloud placeholder already defined. Did you include api.h before api_pcl.h?
#endif //_CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED

#include "cwipc_util/api.h"

_CWIPC_UTIL_EXPORT cwipc *cwipc_from_pcl(cwipc_pcl_pointcloud* pc, char **errorMessage);


#endif // _cwipc_util_api_pcl_h_
