#include <iostream>
#include <fstream>
#include <unistd.h>

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
    cwipc *obj = cwipc_read(argv[1], &message);
    if (obj == NULL) {
        std::cerr << argv[0] << ": Cannot read pointcloud: " << message << std::endl;
        return 1;
    }
    //
    // Save
    //
    int status = cwipc_write_debugdump(argv[2], obj, &message);
    if (status < 0) {
        std::cerr << argv[0] << ": Cannot save pointcloud to cwipcdump: " << message << std::endl;
        return 1;
    }

    return 0;
}

