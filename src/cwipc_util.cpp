#include <cstddef>
#include <stdio.h>

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


class cwipc_auxiliary_data_impl : public cwipc_auxiliary_data {
protected:
    struct item {
        std::string name;
        void *pointer;
        size_t size;
        deallocfunc dealloc;
    };
    std::vector<struct item> m_items;
public:
    cwipc_auxiliary_data_impl() {}
    
    ~cwipc_auxiliary_data_impl() {
        for(auto item: m_items) {
            item.dealloc(item.pointer);
        }
        m_items.clear();
    }
    
    int count() {
        return m_items.size();
    }
    
    const std::string& name(int idx) {
        return m_items[idx].name;
    }
    
    void *pointer(int idx) {
        return m_items[idx].pointer;
    }
    
    size_t size(int idx) {
        return m_items[idx].size;
    }
    
    void _add(const std::string& name, void *pointer, size_t size, deallocfunc dealloc) {
        struct item new_item = {
            .name = name,
            .pointer = pointer,
            .size = size,
            .dealloc = dealloc
        };
        m_items.push_back(new_item);
    }
    
    void _move(cwipc_auxiliary_data *other) {
        auto other_impl = (cwipc_auxiliary_data_impl *)other;
        for(auto item: m_items) {
            other_impl->m_items.push_back(item);
        }
        m_items.clear();
    }
};

class cwipc_impl : public cwipc {
protected:
    uint64_t m_timestamp;
    float m_cellsize;
    cwipc_pcl_pointcloud m_pc;
    cwipc_auxiliary_data* m_aux;
public:
    cwipc_impl() : m_timestamp(0), m_cellsize(0), m_pc(NULL), m_aux(NULL) {}
    cwipc_impl(cwipc_pcl_pointcloud pc, uint64_t timestamp) : m_timestamp(timestamp), m_cellsize(0), m_pc(pc), m_aux(NULL) {}

    ~cwipc_impl() {}

    int from_points(struct cwipc_point *pointData, size_t size, int npoint, uint64_t timestamp)
    {
        if (npoint * sizeof(struct cwipc_point) != size) {
            return -1;
        }
        m_timestamp = timestamp;
        cwipc_pcl_pointcloud pc = new_cwipc_pcl_pointcloud();
        for (int i=0; i<npoint; i++) {
            cwipc_pcl_point point;
            point.x = pointData[i].x;
            point.y = pointData[i].y;
            point.z = pointData[i].z;
            point.r = pointData[i].r;
            point.g = pointData[i].g;
            point.b = pointData[i].b;
            point.a = pointData[i].tile;
            pc->points.push_back(point);
        }
        m_pc = pc;
        return npoint;
    }

    void free() {
        m_pc = NULL;
    }
    
    uint64_t timestamp() {
        return m_timestamp;
    }
    
    float cellsize() {
        return m_cellsize;
    }
    
    void _set_cellsize(float cellsize) {
		if (cellsize < 0 && m_pc) {
			// Guess cellsize by traversing over adjacent points
			// util we find 2 sets with minimum distance.
			float minDistance = std::numeric_limits<float>::infinity();
			auto prevPoint = m_pc->begin();
			for(auto it=m_pc->begin(); it != m_pc->end(); ++it) {
				if (it == prevPoint) continue;
				float distance = pcl::geometry::distance(*it, *prevPoint);
				if (distance < minDistance) {
					minDistance = distance;
#if 0
				} else if (distance == minDistance) {
					break;
#endif
				} /* else continue */
			}
			if (minDistance == std::numeric_limits<float>::infinity()) minDistance = 0;
			cellsize = minDistance;
		}
        m_cellsize = cellsize;
    }
    
	void _set_timestamp(uint64_t timestamp) {
    	m_timestamp = timestamp;
    }

    int count() {
		return m_pc->size();
	}

    size_t get_uncompressed_size() {
        return m_pc->size() * sizeof(struct cwipc_point);
    }
    
    int copy_uncompressed(struct cwipc_point *pointData, size_t size) {
        if (size < m_pc->size() * sizeof(struct cwipc_point)) return -1;
        int npoint = m_pc->size();
        for (int i = 0; i < npoint; i++)
        {
            pointData[i].x = (*m_pc)[i].x;
            pointData[i].y = (*m_pc)[i].y;
            pointData[i].z = (*m_pc)[i].z;
            pointData[i].r = (*m_pc)[i].r;
            pointData[i].g = (*m_pc)[i].g;
            pointData[i].b = (*m_pc)[i].b;
            pointData[i].tile = (*m_pc)[i].a;
        }
        // iterate_over_octree();
        return npoint;
    }
    
    size_t copy_packet(uint8_t *packet, size_t size) {
        size_t dataSize = get_uncompressed_size();
        size_t sizeNeeded = sizeof(struct cwipc_cwipcdump_header) + dataSize;
        if (packet == NULL) {
            return sizeNeeded;
        }
        if (size != sizeNeeded) {
            return 0;
        }
        struct cwipc_cwipcdump_header hdr = { 
            {
                CWIPC_CWIPCDUMP_HEADER[0],
                CWIPC_CWIPCDUMP_HEADER[1],
                CWIPC_CWIPCDUMP_HEADER[2],
                CWIPC_CWIPCDUMP_HEADER[3]
            }, 
            CWIPC_CWIPCDUMP_VERSION, 
            timestamp(), 
            cellsize(),
            0,
            dataSize
        };
        memcpy(packet, &hdr, sizeof(struct cwipc_cwipcdump_header));
        cwipc_point *pointData = (cwipc_point *)(packet + sizeof(struct cwipc_cwipcdump_header));
        copy_uncompressed(pointData, dataSize);
        return sizeNeeded;
    }

    cwipc_pcl_pointcloud access_pcl_pointcloud() {
        return m_pc;
    }
    
    cwipc_auxiliary_data *access_auxiliary_data() {
        if (m_aux == NULL) m_aux = new cwipc_auxiliary_data_impl();
        return m_aux;
    }
};

//
// Another implementation of cwipc: by default stores the
// raw pointcloud data uncompressed, and only copies to a pcl octree
// if needed. Should be much more efficient in workflows where we only
// handle uncompressed data (no need to convert to/from octree representation,
// only do a memcpy).
//
class cwipc_uncompressed_impl : public cwipc_impl {
protected:
    struct cwipc_point *m_points;
    size_t m_points_size;
public:
    cwipc_uncompressed_impl() : cwipc_impl(), m_points(NULL), m_points_size(0) {}
    cwipc_uncompressed_impl(cwipc_pcl_pointcloud pc, uint64_t timestamp) : cwipc_impl(pc, timestamp), m_points(NULL), m_points_size(0) {}
    
    ~cwipc_uncompressed_impl() {
        free();
    }
    
    int from_points(struct cwipc_point *pointData, size_t size, int npoint, uint64_t timestamp)
    {
        if (npoint * sizeof(struct cwipc_point) != size) {
            return -1;
        }
        m_timestamp = timestamp;
        m_points = (struct cwipc_point *)malloc(size);
        if (m_points == NULL) return -1;
        m_points_size = size;
        m_pc = nullptr;
        memcpy(m_points, pointData, size);
        return npoint;
    }
    
    void free() {
        m_pc = NULL;
        if (m_points) ::free(m_points);
        m_points = NULL;
        m_points_size = 0;
    }
    
    uint64_t timestamp() {
        return m_timestamp;
    }
    
    float cellsize() {
        return m_cellsize;
    }
    
    void _set_cellsize(float cellsize) {
        (void)access_pcl_pointcloud();
        cwipc_impl::_set_cellsize(cellsize);
    }
    
    void _set_timestamp(uint64_t timestamp) {
    	m_timestamp = timestamp;
    }
    
    int count() {
        return m_points_size / sizeof(struct cwipc_point);
    }
    
    size_t get_uncompressed_size() {
        return m_points_size;
    }
    
    int copy_uncompressed(struct cwipc_point *pointData, size_t size) {
        if (size != m_points_size) return -1;
        memcpy(pointData, m_points, size);

        return m_points_size / sizeof(struct cwipc_point);
    }
    
    cwipc_pcl_pointcloud access_pcl_pointcloud() {
        if (m_pc == nullptr) {
            cwipc_impl::from_points(m_points, m_points_size, m_points_size / sizeof(struct cwipc_point), m_timestamp);
        }
        return m_pc;
    }
};

cwipc *
cwipc_read(const char *filename, uint64_t timestamp, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_read: incorrect apiVersion";
		}
		return NULL;
	}
    cwipc_pcl_pointcloud pc = new_cwipc_pcl_pointcloud();
    pcl::PLYReader ply_reader;
    if (ply_reader.read(filename, *pc) < 0) {
        if (errorMessage) *errorMessage = (char *)"Error reading ply pointcloud";
        return NULL;
    }
    return new cwipc_impl(pc, timestamp);
}

int 
cwipc_write(const char *filename, cwipc *pointcloud, char **errorMessage)
{
    cwipc_pcl_pointcloud pc = pointcloud->access_pcl_pointcloud();
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
cwipc_read_debugdump(const char *filename, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_read_debugdump: incorrect apiVersion";
		}
		return NULL;
	}
    FILE *fp = fopen(filename, "rb");
    if (fp == NULL) {
        if (errorMessage) *errorMessage = (char *)"Cannot open pointcloud dumpfile";
        return NULL;
    }
    struct cwipc_cwipcdump_header hdr;
    if (fread(&hdr, 1, sizeof(hdr), fp) != sizeof(hdr)) {
        if (errorMessage) *errorMessage = (char *)"Cannot read pointcloud dumpfile header";
        fclose(fp);
        return NULL;
    }
    if (hdr.hdr[0] != CWIPC_CWIPCDUMP_HEADER[0] || hdr.hdr[1] != CWIPC_CWIPCDUMP_HEADER[1] || hdr.hdr[2] != CWIPC_CWIPCDUMP_HEADER[2] || hdr.hdr[3] != CWIPC_CWIPCDUMP_HEADER[3]) {
        if (errorMessage) *errorMessage = (char *)"Pointcloud dumpfile header incorrect";
        fclose(fp);
        return NULL;
    }
    if (hdr.magic != CWIPC_CWIPCDUMP_VERSION) {
        if (errorMessage) *errorMessage = (char *)"Pointcloud dumpfile version incorrect";
        fclose(fp);
        return NULL;
    }
    uint64_t timestamp = hdr.timestamp;
    size_t dataSize = hdr.size;
    float cellsize = hdr.cellsize;
    int npoint = dataSize / sizeof(cwipc_point);
    if (npoint*sizeof(cwipc_point) != dataSize) {
        if (errorMessage) *errorMessage = (char *)"Pointcloud dumpfile datasize inconsistent";
        fclose(fp);
        return NULL;
    }
    cwipc_point* pointData = (cwipc_point *)malloc(dataSize);
    if (pointData == NULL) {
        if (errorMessage) *errorMessage = (char *)"Could not allocate memory for point data";
        fclose(fp);
        return NULL;
    }
    if (fread(pointData, 1, dataSize, fp) != dataSize) {
        if (errorMessage) *errorMessage = (char *)"Could not read point data of correct size";
        fclose(fp);
        return NULL;
    }
    fclose(fp);
    cwipc_uncompressed_impl *pc = new cwipc_uncompressed_impl();
    pc->from_points(pointData, dataSize, npoint, timestamp);
    pc->_set_cellsize(cellsize);
    free(pointData);
	return pc;
}

int 
cwipc_write_debugdump(const char *filename, cwipc *pointcloud, char **errorMessage)
{
    size_t dataSize = pointcloud->get_uncompressed_size();
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
    FILE *fp = fopen(filename, "wb");
    if (fp == NULL) {
        if (errorMessage) *errorMessage = (char *)"Cannot create output file";
        return -1;
    }
    struct cwipc_cwipcdump_header hdr = { 
        {
            CWIPC_CWIPCDUMP_HEADER[0],
            CWIPC_CWIPCDUMP_HEADER[1],
            CWIPC_CWIPCDUMP_HEADER[2],
            CWIPC_CWIPCDUMP_HEADER[3]
        }, 
        CWIPC_CWIPCDUMP_VERSION, 
        pointcloud->timestamp(), 
        pointcloud->cellsize(), 
        0,
        dataSize
    };
    fwrite(&hdr, sizeof(hdr), 1, fp);
    if (fwrite(dataBuf, sizeof(struct cwipc_point), nPoint, fp) != nPoint) {
        if (errorMessage) *errorMessage = (char *)"Write output file failed";
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

cwipc *
cwipc_from_pcl(cwipc_pcl_pointcloud pc, uint64_t timestamp, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_from_pcl: incorrect apiVersion";
		}
		return NULL;
	}
    return new cwipc_impl(pc, timestamp);
}

cwipc *
cwipc_from_points(cwipc_point* points, size_t size, int npoint, uint64_t timestamp, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_from_points: incorrect apiVersion";
		}
		return NULL;
	}
    cwipc_uncompressed_impl *rv = new cwipc_uncompressed_impl();
    if (rv->from_points(points, size, npoint, timestamp) < 0) {
        if (errorMessage) *errorMessage = (char *)"cwipc_from_points: cannot load points (size error?)";
        delete rv;
        return NULL;
    }
    return rv;
}

cwipc *
cwipc_from_packet(uint8_t *packet, size_t size, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
			*errorMessage = (char *)"cwipc_from_packet: incorrect apiVersion";
		}
		return NULL;
	}
    struct cwipc_cwipcdump_header *header = (struct cwipc_cwipcdump_header *) packet;
    struct cwipc_point *points = (struct cwipc_point *)(packet + sizeof(cwipc_cwipcdump_header));
    if (memcmp(header->hdr, CWIPC_CWIPCDUMP_HEADER, 4) != 0 || header->magic != CWIPC_CWIPCDUMP_VERSION) {
        *errorMessage = (char *)"cwipc_from_packet: bad packet header";
        return NULL;
    }
    size_t dataSize = size - sizeof(struct cwipc_cwipcdump_header);
    int npoint = header->size / sizeof(cwipc_point);
    if (npoint * sizeof(cwipc_point) != dataSize) {
        *errorMessage = (char *)"cwipc_from_packet: inconsistent dataSize";
        return NULL;
    }
    cwipc_uncompressed_impl *rv = new cwipc_uncompressed_impl();
    if (rv->from_points(points, dataSize, npoint, header->timestamp) < 0) {
        if (errorMessage) *errorMessage = (char *)"cwipc_from_packet: cannot load points (size error?)";
        delete rv;
        return NULL;
    }
    rv->_set_cellsize(header->cellsize);
    return rv;
}

void cwipc_free(cwipc *pc)
{
    pc->free();
}

uint64_t
cwipc_timestamp(cwipc *pc)
{
    return pc->timestamp();
}

float
cwipc_cellsize(cwipc *pc)
{
    return pc->cellsize();
}

void
cwipc__set_cellsize(cwipc *pc, float cellsize)
{
    pc->_set_cellsize(cellsize);
}

void
cwipc__set_timestamp(cwipc *pc, uint64_t timestamp)
{
    pc->_set_timestamp(timestamp);
}

int
cwipc_count(cwipc *pc)
{
    return pc->count();
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

size_t
cwipc_copy_packet(cwipc *pc, uint8_t *packet, size_t size)
{
    return pc->copy_packet(packet, size);
}

cwipc_auxiliary_data *
cwipc_access_auxiliary_data(cwipc *pc)
{
    return pc->access_auxiliary_data();
}

int cwipc_auxiliary_data_count(cwipc_auxiliary_data *collection) {
    return collection->count();
}

const char * cwipc_auxiliary_data_name(cwipc_auxiliary_data *collection, int idx) {
    return collection->name(idx).c_str();
}
    
void * cwipc_auxiliary_data_pointer(cwipc_auxiliary_data *collection, int idx) {
    return collection->pointer(idx);
}
    
size_t cwipc_auxiliary_data_size(cwipc_auxiliary_data *collection, int idx) {
    return collection->size(idx);
}

cwipc* 
cwipc_source_get(cwipc_source *src)
{
    return src->get();
}

void 
cwipc_source_free(cwipc_source *src)
{
    src->free();
}

bool
cwipc_source_eof(cwipc_source *src)
{
	return src->eof();
}

bool 
cwipc_source_available(cwipc_source *src, bool wait)
{
	return src->available(wait);
}

void
cwipc_source_request_auxiliary_data(cwipc_source *src, const char *name)
{
    src->request_auxiliary_data(name);
}

bool
cwipc_source_auxiliary_data_requested(cwipc_source *src, const char *name)
{
    return src->auxiliary_data_requested(name);
}

int
cwipc_tiledsource_maxtile(cwipc_tiledsource *src)
{
	return src->maxtile();
}

bool
cwipc_tiledsource_get_tileinfo(cwipc_tiledsource *src, int tilenum, struct cwipc_tileinfo *tileinfo)
{
	return src->get_tileinfo(tilenum, tileinfo);
}

void cwipc_sink_free(cwipc_sink *sink) {
    sink->free();
}

bool cwipc_sink_feed(cwipc_sink *sink, cwipc *pc, bool clear) {
    return sink->feed(pc, clear);
}

bool cwipc_sink_caption(cwipc_sink *sink, const char *caption) {
    return sink->caption(caption);
}

char cwipc_sink_interact(cwipc_sink *sink, const char *prompt, const char *responses, int32_t millis) {
    return sink->interact(prompt, responses, millis);
}
