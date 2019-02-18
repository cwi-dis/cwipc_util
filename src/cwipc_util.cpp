#include <cstddef>
#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>


#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"

class cwipc_impl : public cwipc {
protected:
    cwipc_pcl_pointcloud *m_pc;
public:
    cwipc_impl() : m_pc(NULL) {}
    cwipc_impl(cwipc_pcl_pointcloud *pc) : m_pc(pc) {}
    ~cwipc_impl() {};
    void free() {
        delete m_pc;
    }
    
    uint32_t timestamp() {
        return 0;
    }
    
    size_t get_uncompressed_size() {
        return m_pc->size() * sizeof(struct cwipc_point);
    }
    
    int copy_uncompressed(struct cwipc_point *pointData, size_t size) {
        if (size < get_uncompressed_size()) return -1;
        for (int i = 0; i < m_pc->size(); i++)
        {
            pointData[i].x = (*m_pc)[i].x;
            pointData[i].y = (*m_pc)[i].y;
            pointData[i].z = (*m_pc)[i].z;
            pointData[i].r = (*m_pc)[i].r;
            pointData[i].g = (*m_pc)[i].g;
            pointData[i].b = (*m_pc)[i].b;
        }
        // iterate_over_octree();
        return -1;
    }
    
    cwipc_pcl_pointcloud *access_pcl_pointcloud() {
        return m_pc;
    }
};

cwipc *
cwipc_read(const char *filename, char **errorMessage)
{
    pcl::PointCloud<pcl::PointXYZRGB> *pc = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PLYReader ply_reader;
    if (ply_reader.read(filename, *pc) < 0) {
        if (errorMessage) *errorMessage = (char *)"Error reading ply pointcloud";
        return NULL;
    }
    return new cwipc_impl(pc);
}

int 
cwipc_write(const char *filename, cwipc *pointcloud, char **errorMessage)
{
    pcl::PointCloud<pcl::PointXYZRGB> *pc = pointcloud->access_pcl_pointcloud();
    if (pc == NULL) {
        if (errorMessage) *errorMessage = (char *)"Not yet implemented";
        return -1;
    }
    pcl::PLYWriter writer;
    int status = writer.write(filename, *pc);
    if (status < 0) {
        if (errorMessage) *errorMessage = (char *)"Saving of PLY file failed";
    }
	return status;
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
    int dataSize = pointcloud->get_uncompressed_size();
    struct cwipc_point *dataBuf = (struct cwipc_point *)malloc(dataSize);
    if (dataBuf == NULL) {
        if (errorMessage) *errorMessage = (char *)"Cannot allocate pointcloud memory";
        return -1;
    }
    int nPoint = pointcloud->copy_uncompressed(dataBuf, dataSize);
    if (nPoint < 0) {
        if (errorMessage) *errorMessage = (char *)"Cannot copy points from pointcloud";
        return -1;
    }
    FILE *fp = fopen(filename, "w");
    if (fp == NULL) {
        if (errorMessage) *errorMessage = (char *)"Cannot create output file";
        return -1;
    }
    fwrite(dataBuf, sizeof(struct cwipc_point), nPoint, fp);
    fclose(fp);
    return 0;
}

cwipc *
cwipc_from_pcl(cwipc_pcl_pointcloud* pc, char **errorMessage)
{
    return new cwipc_impl(pc);
}

void cwipc_free(cwipc *pc)
{
    pc->free();
}

uint32_t
cwipc_timestamp(cwipc *pc)
{
    return pc->timestamp();
}

size_t
cwipc_get_uncompressed_size(cwipc *pc)
{
    return pc->get_uncompressed_size();
}

int
cwipc_copy_uncompressed(cwipc *pc, struct cwipc_point *points, size_t size)
{
    return pc->copy_uncompressed(points, size);
}
