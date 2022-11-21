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

class cwipc_source_synthetic_impl : public cwipc_tiledsource {
private:
    float m_angle;
    std::chrono::system_clock::time_point m_start;
    std::chrono::system_clock::time_point m_earliest_next_pointcloud;
    int m_hsteps;
    int m_asteps;
    int m_fps;
    cwipc_point *m_points;
    size_t m_points_size;
public:
    cwipc_source_synthetic_impl(int fps=0, int npoints=0)
    : m_angle(0),
      m_start(std::chrono::system_clock::now()),
      m_earliest_next_pointcloud(),
      m_hsteps(0),
      m_asteps(0),
      m_fps(fps),
      m_points(NULL),
      m_points_size(0)
    {
    	if (npoints == 0) npoints = 160000;
    	m_hsteps = m_asteps = int(sqrt(npoints));
        npoints = m_hsteps * m_asteps;
        m_points_size = npoints*sizeof(cwipc_point);
    	m_points = (cwipc_point *)malloc(m_points_size);
    }

    ~cwipc_source_synthetic_impl() {
    	free();
    }

    void free() {
    	if (m_points) ::free(m_points);
    	m_points = NULL;
    }
    
    bool eof() {
    	return false;
    }
    
    bool available(bool wait) {
    	return true;
    }

    cwipc* get() {
        if (m_fps != 0 && m_earliest_next_pointcloud.time_since_epoch() != std::chrono::milliseconds(0)) {
            std::this_thread::sleep_until(m_earliest_next_pointcloud);
        }
		uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::chrono::duration<float, std::ratio<1>> runtime = std::chrono::system_clock::now() - m_start;
        if (m_fps != 0) {
            m_earliest_next_pointcloud = std::chrono::system_clock::now() + std::chrono::milliseconds(1000 / m_fps);
        }
        m_angle = runtime.count();
        generate_points();
        cwipc *rv = cwipc_from_points(m_points, m_points_size, m_hsteps*m_asteps, timestamp, NULL, CWIPC_API_VERSION);
        if (rv) {
            rv->_set_cellsize(2.0 / m_hsteps);
            // For testing purposes: save angle if wanted
            if ( auxiliary_data_requested("test-angle")) {
                void *memptr = malloc(sizeof(m_angle));
                memcpy(memptr, &m_angle, sizeof(m_angle));
                cwipc_auxiliary_data *ap = rv->access_auxiliary_data();
                ap->_add("test-angle", "", memptr, sizeof(m_angle), ::free);
            }
        }
        return rv;
    }

	int maxtile() { return 3; }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
    	static cwipc_tileinfo syntheticInfo[3] = {
    		{{0, 0, 0}, (char *)"synthetic", 2, 0},
    		{{0, 0, 1}, (char *)"synthetic-right", 1, 1},
    		{{0, 0, -1}, (char *)"synthetic-left", 1, 2},
		};
		switch(tilenum) {
		case 0:
		case 1:
		case 2:
			if (tileinfo) *tileinfo = syntheticInfo[tilenum];
			return true;
		}
		return false;
	}

private:
	void generate_points()
	{
        const float  pi=3.14159265358979f;
        const float max_height = 2.0;
        const float delta_h = max_height / m_hsteps;
        const float delta_a = 2*pi / m_asteps;
        cwipc_point *pptr = m_points;
        for (int height_i=0; height_i < m_hsteps; height_i++) {
            float height = height_i * delta_h;
            for (int angle_i=0; angle_i < m_asteps; angle_i++) {
                float angle = angle_i * delta_a;
                float radius = 0.3* pow(cos(height*pi/3-pi/6), 0.71);
                float x = radius*sin(angle);
                float y = radius*cos(angle);
                float r = (1+sin(2*pi*height+m_angle+angle))/2;
                float g = (1+sin(3*pi*height+m_angle+angle))/2;
                float b = (1+sin(4*pi*height+m_angle+angle))/2;
                int rr = (int)(r*255.0);
                int gg = (int)(g*255.0);
                int bb = (int)(b*255.0);
                // Eyes
                if (height > 1.7 && height < 1.8 && ((angle > pi*0.083 && angle < pi*0.1667) || (angle > pi*1.833 && angle < pi*1.917))) {
                    if (fmod(m_angle, pi/2) > 0.08) {
                        rr = gg = bb = 255;
                    }
                }
                pptr->x = -x;
                pptr->y = height;
                pptr->z = y;
                pptr->r = rr;
                pptr->g = gg;
                pptr->b = bb;
                pptr->tile = y < 0 ? 1 : 2;
                pptr++;
            }
        }
	}

};

cwipc_tiledsource *
cwipc_synthetic(int fps, int npoints, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_synthetic: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }
		return NULL;
	}
	return new cwipc_source_synthetic_impl(fps, npoints);
}
