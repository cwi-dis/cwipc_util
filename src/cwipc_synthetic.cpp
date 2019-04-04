#include <chrono>
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

class cwipc_source_synthetic_impl : public cwipc_source {
private:
    float m_angle;
    cwipc_pcl_pointcloud m_pointcloud;
public:
    cwipc_source_synthetic_impl() : m_angle(0), m_pointcloud(0) {
    	generate_pcl();
    }

    ~cwipc_source_synthetic_impl() {
        m_pointcloud = NULL;
    }

    void free() {
        m_pointcloud = NULL;
    }
    
    bool eof() {
    	return false;
    }
    
    bool available(bool wait) {
    	return true;
    }

    cwipc* get() {
        if (m_pointcloud == NULL) return NULL;
		uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		m_angle += 0.031415;
		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(Eigen::AngleAxisf(m_angle, Eigen::Vector3f::UnitY()));
		cwipc_pcl_pointcloud newPC = new_cwipc_pcl_pointcloud();
		transformPointCloud(*m_pointcloud, *newPC, transform);
        return cwipc_from_pcl(newPC, timestamp, NULL);
    }
    
private:
	void generate_pcl()
	{
		m_pointcloud = new_cwipc_pcl_pointcloud();
		uint8_t r(255), g(15), b(15);
		for (float z(-1.0f); z <= 1.0f; z += 0.005f) {
			float angle(0.0);
			while (angle <= 360.0) {
				cwipc_pcl_point point;
				point.x = 0.5f*cosf(pcl::deg2rad(angle))*(1.0f - z * z);
				point.y = sinf(pcl::deg2rad(angle))*(1.0f - z * z);
				point.z = z;
				uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				m_pointcloud->points.push_back(point);
				float r = sqrt(point.x*point.x + point.y*point.y);
				if (r > 0.0)
					angle += 0.27 / r;
				else break;
			}
			if (z < 0.0) { r -= 1; g += 1; }
			else { g -= 1; b += 1; }
		}
		m_pointcloud->width = (int)m_pointcloud->points.size();
		m_pointcloud->height = 1;
	}

};

cwipc_source *
cwipc_synthetic()
{
	return new cwipc_source_synthetic_impl();
}
