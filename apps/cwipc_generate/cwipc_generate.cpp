#include <iostream>
#include <fstream>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " count directory" << std::endl;
        std::cerr << "Creates COUNT synthetic pointclouds and stores the PLY files in the given DIRECTORY" << std::endl;
        return 2;
    }
    int count = atoi(argv[1]);
    char filename[500];
    char *error;
    
    cwipc_source *generator = cwipc_synthetic(0, 0, &error, CWIPC_API_VERSION);
    if (generator == NULL) {
    	std::cerr << "Error: " << error << std::endl;
    	return 1;
    }
    int ok = 0;
    while (count-- > 0 && ok == 0) {
    	cwipc *pc = generator->get();
    	snprintf(filename, sizeof(filename), "%s/pointcloud-%llu.ply", argv[2], pc->timestamp());
    	ok = cwipc_write(filename, pc, &error);
        pc->free();
    }
    if (ok < 0) {
    	std::cerr << "Error: " << error << std::endl;
    	return 1;
    }
    return 0;
}

