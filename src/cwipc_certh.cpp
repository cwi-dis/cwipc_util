#include <cstddef>
#include <stdio.h>
#include <inttypes.h>

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

struct CerthPointCloud {
    int numDevices;
    float **vertexPtr;
    float **normalPtr;
    uint8_t **colorPtr;
    char *deviceNames;
    int *verticesPerCamera;
    void *vertexChannels;
    void *normalChannels;
    void *colorChannels;
    void *pclData;
};

cwipc* cwipc_from_certh(void* certhPC, float *origin, float *bbox, uint64_t timestamp, char **errorMessage, uint64_t apiVersion) {
    if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
        if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_from_certh: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }

        return NULL;
    }

    struct CerthPointCloud *cpc = (struct CerthPointCloud *)certhPC;
    cwipc_pcl_pointcloud pclPC = new_cwipc_pcl_pointcloud();

    for (int camNum=0; camNum <cpc->numDevices; camNum++) {
        int numVertices = cpc->verticesPerCamera[camNum];
        float *vertices = cpc->vertexPtr[camNum];
        uint8_t *colors = cpc->colorPtr[camNum];

        for (int vertexNum=0; vertexNum < numVertices; vertexNum++) {
            float x = vertices[4*vertexNum+0];
            float y = vertices[4*vertexNum+1];
            float z = vertices[4*vertexNum+2];

            if (origin) {
                x -= origin[0];
                y -= origin[1];
                z -= origin[2];
            }

            // Fourth coordinate is W which is weight, which is currently unused.
            uint8_t r = colors[3*vertexNum+2];
            uint8_t g = colors[3*vertexNum+1];
            uint8_t b = colors[3*vertexNum+0];
            uint8_t tile = (1<<camNum);

            if (bbox) {
                if (x < bbox[0] || x > bbox[1]) {
                    continue;
                }

                if (y < bbox[2] || y > bbox[3]) {
                    continue;
                }

                if (z < bbox[4] || z > bbox[5]) {
                    continue;
                }
            }

            cwipc_pcl_point point;
            point.x = x;
            point.y = y;
            point.z = z;
            point.r = r;
            point.g = g;
            point.b = b;
            point.tile = tile;
            pclPC->points.push_back(point);
        }
    }

    return cwipc_from_pcl(pclPC, timestamp, errorMessage, apiVersion);
}
