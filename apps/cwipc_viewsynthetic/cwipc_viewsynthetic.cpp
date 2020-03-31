#include <iostream>
#include <fstream>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
    if (argc != 1) {
        std::cerr << "Usage: " << argv[0]  << std::endl;
        std::cerr << "Create synthetic pointclouds and show them in a window" << std::endl;
        return 2;
    }
    char *error;
    
    cwipc_source *generator = cwipc_synthetic(&error, CWIPC_API_VERSION);
    if (generator == NULL) {
    	std::cerr << "Error: " << error << std::endl;
    	return 1;
    }
    cwipc_sink *window = cwipc_window("cwipc_viewsynthetic", &error, CWIPC_API_VERSION);
    if (window == NULL) {
        std::cerr << "Error: " << error << std::endl;
        return 1;
    }
    while (true) {
    	cwipc *pc = generator->get();
        bool ok = window->feed(pc, true);
        if (!ok) {
            std::cerr << "Error: window->feed() returned false" << std::endl;
            return 1;
        }
        pc->free();
        char response = window->interact("Type q to quit", "q", 30);
        if (response == 'q') break;
    }
    window->free();
    generator->free();
    return 0;
}

