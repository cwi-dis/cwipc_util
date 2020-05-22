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

class cwipc_source_synthetic_impl : public cwipc_tiledsource {
private:
    float m_angle;
    std::chrono::system_clock::time_point m_start;
    static const int H_STEPS = 400;
    static const int A_STEPS = 400;
    cwipc_point m_points[H_STEPS*A_STEPS];
public:
    cwipc_source_synthetic_impl()
    : m_angle(0),
      m_start(std::chrono::system_clock::now())
    {
    }

    ~cwipc_source_synthetic_impl() {
    }

    void free() {
    }
    
    bool eof() {
    	return false;
    }
    
    bool available(bool wait) {
    	return true;
    }

    cwipc* get() {
		uint64_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        std::chrono::duration<float, std::ratio<1>> runtime = std::chrono::system_clock::now() - m_start;
        
        m_angle = runtime.count();
        generate_points();
        cwipc *rv = cwipc_from_points(m_points, sizeof(m_points), H_STEPS*A_STEPS, timestamp, NULL, CWIPC_API_VERSION);
        rv->_set_cellsize(0.01);
        return rv;
    }

	int maxtile() { return 3; }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
    	static cwipc_tileinfo syntheticInfo[3] = {
    		{{0, 0, 0}, NULL, 0},
    		{{0, 0, -1}, NULL, 0},
    		{{0, 0, 1}, NULL, 0},
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
        const float delta_h = max_height / H_STEPS;
        const float delta_a = 2*pi / A_STEPS;
        cwipc_point *pptr = m_points;
        for (int height_i=0; height_i < H_STEPS; height_i++) {
            float height = height_i * delta_h;
            for (int angle_i=0; angle_i < A_STEPS; angle_i++) {
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
                pptr->x = x;
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
cwipc_synthetic(char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_synthetic: incorrect apiVersion";
		}
		return NULL;
	}
	return new cwipc_source_synthetic_impl();
}
