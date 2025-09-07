#ifndef _cwipc_util_api_pcl_h_
#define _cwipc_util_api_pcl_h_

#ifndef __cplusplus
#error "api_pcl.h requires C++"
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#ifndef _CWIPC_PCL_POINTCLOUD_DEFINED
struct _cwipc_bitmask {
//    union {
//        struct {
//            bool b0 : 1;
//            bool b1 : 1;
//            bool b2 : 1;
//            bool b3 : 1;
//            bool b4 : 1;
//            bool b5 : 1;
//            bool b6 : 1;
//            bool b7 : 1;
//        };
        uint8_t allbits;
//    };
};

struct cwipc_bitmask : public _cwipc_bitmask {
    // inline cwipc_bitmask() { allbits = 0; }
    // inline cwipc_bitmask(uint8_t value) { allbits = value; }
    inline uint8_t getvalue() { return allbits; }
//    cwipc_bitmask& operator=(const cwipc_bitmask& other) { allbits = other.allbits; return *this; }
    cwipc_bitmask& operator=(uint8_t value) { allbits = value; return *this; }
    cwipc_bitmask& operator|=(const cwipc_bitmask& other) { allbits |= other.allbits; return *this; }
//    operator uint8_t() {return allbits; }
};

struct EIGEN_ALIGN16 _PointXYZRGBTile
{
    PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed using the point (which is float[4])
      union
      {
        union
        {
          struct
          {
            std::uint8_t b;
            std::uint8_t g;
            std::uint8_t r;
            cwipc_bitmask tile;
          };
          float rgb;
        };
        std::uint32_t rgbtile;
      };

    // inline Eigen::Vector3i getRGBVector3i () { return (Eigen::Vector3i (r, g, b)); }
    // inline const Eigen::Vector3i getRGBVector3i () const { return (Eigen::Vector3i (r, g, b)); }
    // inline Eigen::Vector4i getRGBVector4i () { return (Eigen::Vector4i (r, g, b, a)); }
    // inline const Eigen::Vector4i getRGBVector4i () const { return (Eigen::Vector4i (r, g, b, a)); }
    // inline Eigen::Vector4i getRGBAVector4i () { return (Eigen::Vector4i (r, g, b, a)); }
    // inline const Eigen::Vector4i getRGBAVector4i () const { return (Eigen::Vector4i (r, g, b, a)); }
    // inline pcl::Vector3cMap getBGRVector3cMap () { return (pcl::Vector3cMap (reinterpret_cast<std::uint8_t*> (&rgbtile))); }
    // inline pcl::Vector3cMapConst getBGRVector3cMap () const { return (pcl::Vector3cMapConst (reinterpret_cast<const std::uint8_t*> (&rgbtile))); } \
    // inline pcl::Vector4cMap getBGRAVector4cMap () { return (pcl::Vector4cMap (reinterpret_cast<std::uint8_t*> (&rgba))); } \
    // inline pcl::Vector4cMapConst getBGRAVector4cMap () const { return (pcl::Vector4cMapConst (reinterpret_cast<const std::uint8_t*> (&rgba))); }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointXYZRGBTile : public _PointXYZRGBTile
{
    inline PointXYZRGBTile (const _PointXYZRGBTile &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        rgbtile = p.rgbtile;
    }
    
    inline PointXYZRGBTile ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        r = g = b = 0;
        tile = 0;
    }
    
    friend std::ostream& operator << (std::ostream& os, const PointXYZRGBTile& p);
};

PCL_EXPORTS std::ostream& operator << (std::ostream& os, const PointXYZRGBTile& p);

POINT_CLOUD_REGISTER_POINT_STRUCT (_PointXYZRGBTile,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (std::uint32_t, rgbtile, rgbtile)
                                   )
POINT_CLOUD_REGISTER_POINT_WRAPPER(PointXYZRGBTile, _PointXYZRGBTile)

/** \brief PCL point, as supported by this library.
 */
typedef PointXYZRGBTile cwipc_pcl_point;

/** \brief PCL Pointcloud, as supported by this library.
 */
typedef  pcl::shared_ptr<pcl::PointCloud<cwipc_pcl_point>> cwipc_pcl_pointcloud;

/** \brief Allocate an empty cwipc_pcl_pointcloud.
 */
inline cwipc_pcl_pointcloud new_cwipc_pcl_pointcloud(void) { return cwipc_pcl_pointcloud(new pcl::PointCloud<cwipc_pcl_point>); }
#define _CWIPC_PCL_POINTCLOUD_DEFINED
#endif //_CWIPC_PCL_POINTCLOUD_DEFINED

#ifdef _CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED
#error cwipc_pcl_pointcloud placeholder already defined. Did you include api.h before api_pcl.h?
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
