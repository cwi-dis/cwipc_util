#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
    char *message = NULL;
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "pointcloudfile.cwipcdump pointcloudfile.ply" << std::endl;
        return 2;
    }
    //
    // Read pointcloud file
    //
    cwipc *obj = cwipc_read_debugdump(argv[1], &message, CWIPC_API_VERSION);
    if (obj == NULL) {
        std::cerr << argv[0] << ": Cannot read pointcloud from dump: " << message << std::endl;
        return 1;
    }
    //
    // Save
    //
    if (strcmp(argv[2], "-") == 0) {
        // copy-uncompressed in stead of save (for performance testing)
        size_t nbytes = obj->get_uncompressed_size();
        cwipc_point *points = (cwipc_point *)malloc(nbytes);
        if (points == NULL) {
            std::cerr << argv[0] << ": out of memory" << std::endl;
            return 1;
        }
        obj->copy_uncompressed(points, nbytes);
        std::cerr << argv[0] << ": Skipping save" << std::endl;
    } else {
        int status = cwipc_write(argv[2], obj, &message);
        if (status < 0) {
            std::cerr << argv[0] << ": Cannot save pointcloud to ply: " << message << std::endl;
            return 1;
        }
    }
    return 0;
}

