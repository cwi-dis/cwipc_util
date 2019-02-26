#include <iostream>
#include <fstream>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
    char *message = NULL;
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << "count directory" << std::endl;
        std::cerr << "Creates COUNT synthetic pointclouds and stores the PLY files in the given DIRECTORY";
        return 2;
    }
    int count = atoi(argv[1]);
    char filename[500];
    char *error;
    
    cwipc_source *generator = cwipc_synthetic();
    int ok = 0;
    while (count-- > 0 && ok == 0) {
    	cwipc *pc = generator->get();
    	snprintf(filename, sizeof(filename), "%s/pointcloud-%lld.ply", argv[2], pc->timestamp());
    	ok = cwipc_write(filename, pc, &error);
    }
    if (ok < 0) {
    	std::cerr << "Error: " << error << std::endl;
    	return 1;
    }
    return 0;
}

