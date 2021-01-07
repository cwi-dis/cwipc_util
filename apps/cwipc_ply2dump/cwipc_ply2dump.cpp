#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
    char *message = NULL;
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "pointcloudfile.ply pointcloudfile.cwipcdump" << std::endl;
        return 2;
    }
    //
    // Read pointcloud file
    //
    cwipc *obj = cwipc_read(argv[1], 0, &message, CWIPC_API_VERSION);
    if (obj == NULL) {
        // copy-uncompressed in stead of save (for performance testing)
        size_t nbytes = obj->get_uncompressed_size();
        cwipc_point *points = (cwipc_point *)malloc(nbytes);
        if (points == NULL) {
            std::cerr << argv[0] << ": out of memory" << std::endl;
            return 1;
        }
        obj->copy_uncompressed(points, nbytes);
        std::cerr << argv[0] << ": Cannot read pointcloud: " << message << std::endl;
        return 1;
    }
    //
    // Save
    //
    if (strcmp(argv[2], "-") == 0) {
        std::cerr << argv[0] << ": Skipping save" << std::endl;
    } else {
        int status = cwipc_write_debugdump(argv[2], obj, &message);
        if (status < 0) {
            std::cerr << argv[0] << ": Cannot save pointcloud to cwipcdump: " << message << std::endl;
            return 1;
        }
    }
    return 0;
}

