#include <iostream>
#include <fstream>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
	uint64_t timestamp = 0LL;
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " kNeighbors stddevMulThresh pointcloudfile.ply newpointcloudfile.ply" << std::endl;
        std::cerr << "Example: " << argv[0] << " 20 1.0 pointcloudfile.ply newpointcloudfile.ply" << std::endl;
        return 2;
    }
	int kNeighbors = atoi(argv[1]);
    float stddevMulThresh = atof(argv[2]);
    //
    // Read pointcloud file
    //
    char *errorMessage = NULL;
    cwipc *pc = cwipc_read(argv[3], 0LL, &errorMessage, CWIPC_API_VERSION);

    if (pc == NULL || errorMessage) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[3] << ": " << errorMessage << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc->get_uncompressed_size() << " bytes (uncompressed)" << std::endl;
    //
    // Voxelize
    //
    cwipc *new_pc = cwipc_remove_outliers(pc, kNeighbors, stddevMulThresh);
    //
    // Save pointcloud file
    //
    if (cwipc_write(argv[4], new_pc, NULL) < 0) {
    	std::cerr << argv[0] << ": Error writing PLY file " << argv[4] << std::endl;
    	return 1;
    }
    pc->free();
    new_pc->free();

    return 0;
}

