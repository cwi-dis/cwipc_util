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
    int status = cwipc_write(argv[2], obj, &message);
    if (status < 0) {
        std::cerr << argv[0] << ": Cannot save pointcloud to ply: " << message << std::endl;
        return 1;
    }
    return 0;
}

