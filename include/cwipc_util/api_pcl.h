#ifndef _cwipc_util_api_pcl_h_
#define _cwipc_util_api_pcl_h_

#ifndef __cplusplus
#error "api_pcl.h requires C++"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef _CWIPC_PCL_POINTCLOUD_DEFINED
/** \brief PCL point, as supported by this library.
 */
typedef pcl::PointXYZRGB cwipc_pcl_point;

/** \brief PCL Pointcloud, as supported by this library.
 */
typedef  boost::shared_ptr<pcl::PointCloud<cwipc_pcl_point>> cwipc_pcl_pointcloud;

/** \brief Allocate an empty cwipc_pcl_pointcloud.
 */
inline cwipc_pcl_pointcloud new_cwipc_pcl_pointcloud(void) { return cwipc_pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>); }
#define _CWIPC_PCL_POINTCLOUD_DEFINED
#endif //_CWIPC_PCL_POINTCLOUD_DEFINED

#ifdef _CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED
#warning cwipc_pcl_pointcloud placeholder already defined. Did you include api.h before api_pcl.h?
#endif //_CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED

#include "cwipc_util/api.h"

/** \brief Create cwipc pointcloud from PCL pointcloud.
 * \param pc PCL pointcloud.
 * \param timestamp The timestamp to record in the cwipc object.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */ 
_CWIPC_UTIL_EXPORT cwipc *cwipc_from_pcl(cwipc_pcl_pointcloud pc, uint64_t timestamp, char **errorMessage);


#endif // _cwipc_util_api_pcl_h_
