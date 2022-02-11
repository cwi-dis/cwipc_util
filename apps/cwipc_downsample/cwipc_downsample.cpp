#include <iostream>
#include <fstream>

// Define PERFORMANCE_TEST_COUNT greater than 1 to do the downsample multiple times to test performance
#define PERFORMANCE_TEST_COUNT 10

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
	uint64_t timestamp = 0LL;
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " voxelsize pointcloudfile.ply newpointcloudfile.ply" << std::endl;
        return 2;
    }
	float voxelsize = atof(argv[1]);
    //
    // Read pointcloud file
    //
    char *errorMessage = NULL;
    cwipc *pc = cwipc_read(argv[2], 0LL, &errorMessage, CWIPC_API_VERSION);

    if (pc == NULL || errorMessage) {
        std::cerr << argv[0] << ": Error reading pointcloud from " << argv[2] << ": " << errorMessage << std::endl;
        return 1;
    }
    std::cerr << "Read pointcloud successfully, " << pc->get_uncompressed_size() << " bytes (uncompressed), " << pc->count() << " points" << std::endl;
    //
    // Voxelize
    //
    auto t0_wall = std::chrono::high_resolution_clock::now();
    cwipc *new_pc = nullptr;
    for(int i=0; i < PERFORMANCE_TEST_COUNT; i++) {
        if (new_pc != nullptr) new_pc->free();
        new_pc = cwipc_downsample(pc, voxelsize);
    }
    auto t1_wall = std::chrono::high_resolution_clock::now();
    
    double delta_wall = std::chrono::duration<double, std::milli>(t1_wall - t0_wall).count();
	delta_wall /= PERFORMANCE_TEST_COUNT;
    std::cerr << argv[0] << ": did " << PERFORMANCE_TEST_COUNT << " downsamples, " << delta_wall << " ms per call, " << new_pc->count() << " points" << std::endl;
    // Save pointcloud file
    //
    if (cwipc_write(argv[3], new_pc, NULL) < 0) {
    	std::cerr << argv[0] << ": Error writing PLY file " << argv[3] << std::endl;
    	return 1;
    }
    pc->free();
    new_pc->free();

    return 0;
}

