#include <iostream>
#include <fstream>
#include "string.h"
#include <stdlib.h>
#include <inttypes.h>

const std::string libExecDir(LIBEXECDIR);
std::string progName;

int check() {

    std::string cmd;
    int status;
    bool ok = true;
    
    cmd = libExecDir + "/cwipc/cwipc_util_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << progName << ": " << cmd << ": exit status: " << status << std::endl;
    
    cmd = libExecDir + "/cwipc/cwipc_codec_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << progName << ": " << cmd << ": exit status: " << status << std::endl;
    
    cmd = libExecDir + "/cwipc/cwipc_realsense2_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << progName << ": " << cmd << ": exit status: " << status << std::endl;
    
    cmd = libExecDir + "/cwipc/cwipc_kinect_install_check";
    status = ::system(cmd.c_str());
    if (status != 0) ok = false;
    std::cerr << progName << ": " << cmd << ": exit status: " << status << std::endl;

    if (!ok) return 1;
    return 0;
}

int install() {
#ifdef WIN32
    std::string script = libExecDir + "\\cwipc\\scripts\\install-3rdparty-full-win1064-asadmin.ps1";

    std::string cmd = "powershell -ExecutionPolicy Bypass -File " + script;
    std::cerr << progName << ": execute command: " << cmd << std::endl;
    int status = ::system(cmd.c_str());
    return status;
#else
    std::cerr << progName << ": only implemented on Windows. On other platforms use your local package manager (brew, apt, etc)" << std::endl;
    return -1;
#endif
}

int main(int argc, char** argv) {
    progName = argv[0];

    std::string command = "check";
    if (argc >= 2) {
        command = argv[1];
    }
    if (command == "check") {
        int sts = check();
        if (sts != 0) {
            std::cerr << progName << ": Maybe running \"cwipc_check install\" can fix things" << std::endl;
            return sts;
        }
    }
    else if (command == "install") {
        int sts = install();
        std::cerr << progName << ": install: exit status " << sts << std::endl;
        return sts;
    }
    else {
        std::cerr << "Usage: " << progName << "[ check | install | help ]" << std::endl;
        std::cerr << "check   - Tries to determine whether all cwipc third party requirements have been installed correctly (default option)" << std::endl;
        std::cerr << "install - Tries to install all cwipc third party requirements." << std::endl;
        std::cerr << "          This should open a PowerShell window as Administrator. If anything goes wrong and you want to submit a bug report" << std::endl;
        std::cerr << "          please include the full content of this PowerShell window." << std::endl;
        std::cerr << "help    - this message" << std::endl << std::endl;
    }
}
