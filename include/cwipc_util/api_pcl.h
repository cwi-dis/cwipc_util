#ifndef _cwipc_util_api_pcl_h_
#define _cwipc_util_api_pcl_h_

#ifndef __cplusplus
#error "api_pcl.h requires C++"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef _CWIPC_PCL_POINTCLOUD_DEFINED
struct EIGEN_ALIGN16 _PointXYZRGBMask
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
    PCL_ADD_RGB;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZRGBMask : public _PointXYZRGBMask
{
    inline PointXYZRGBMask (const _PointXYZRGBMask &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        rgba = p.rgba;
    }
    
    inline PointXYZRGBMask ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        r = g = b = 0;
        a = 0;
    }
    
    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBMask& p);
};

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBMask& p);

POINT_CLOUD_REGISTER_POINT_STRUCT (_PointXYZRGBMask,           // here we assume a XYZ + "test" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (uint32_t, rgba, rgba)
                                   )
POINT_CLOUD_REGISTER_POINT_WRAPPER(PointXYZRGBMask, _PointXYZRGBMask)

/** \brief PCL point, as supported by this library.
 */
typedef PointXYZRGBMask cwipc_pcl_point;

/** \brief PCL Pointcloud, as supported by this library.
 */
typedef  boost::shared_ptr<pcl::PointCloud<cwipc_pcl_point>> cwipc_pcl_pointcloud;

/** \brief Allocate an empty cwipc_pcl_pointcloud.
 */
inline cwipc_pcl_pointcloud new_cwipc_pcl_pointcloud(void) { return cwipc_pcl_pointcloud(new pcl::PointCloud<cwipc_pcl_point>); }
#define _CWIPC_PCL_POINTCLOUD_DEFINED
#endif //_CWIPC_PCL_POINTCLOUD_DEFINED

#ifdef _CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED
#warning cwipc_pcl_pointcloud placeholder already defined. Did you include api.h before api_pcl.h?
#endif //_CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED

#include "cwipc_util/api.h"

/** \brief Create cwipc pointcloud from PCL pointcloud.
 * \param pc PCL pointcloud.
 * \param timestamp The timestamp to record in the cwipc object.
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */ 
_CWIPC_UTIL_EXPORT cwipc *cwipc_from_pcl(cwipc_pcl_pointcloud pc, uint64_t timestamp, char **errorMessage, uint64_t apiVersion);


#endif // _cwipc_util_api_pcl_h_
