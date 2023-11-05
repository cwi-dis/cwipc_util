#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>

const std::string libExecDir(LIBEXECDIR);

int main(int argc, char** argv) {
    std::cerr << argv[0] << ": LIBEXECDIR: " << libExecDir << std::endl;
    std::string cmd;
    int status;
    bool ok = true;
    
    cmd = libExecDir + "/cwipc_util_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << argv[0] << ": " << cmd << ": exist status: " << status << std::endl;
    
    cmd = libExecDir + "/cwipc_codec_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << argv[0] << ": " << cmd << ": exist status: " << status << std::endl;
    
    cmd = libExecDir + "/cwipc_realsense2_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << argv[0] << ": " << cmd << ": exist status: " << status << std::endl;
    
    cmd = libExecDir + "/cwipc_kinect_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << argv[0] << ": " << cmd << ": exist status: " << status << std::endl;

    if (!ok) return 1;
    return 0;
}

