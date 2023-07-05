#include <chrono>
#include <thread>
#include <inttypes.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include "cwipc_util/internal.h"

struct capturer {
    std::string name;
    _cwipc_functype_count_devices* countFunc;
    _cwipc_func_capturer_factory* factoryFunc;
};

std::vector<struct capturer> all_capturers;

cwipc_tiledsource *
cwipc_capturer(const char *configFilename, char **errorMessage, uint64_t apiVersion)
{
	if (apiVersion < CWIPC_API_VERSION_OLD || apiVersion > CWIPC_API_VERSION) {
		if (errorMessage) {
            char* msgbuf = (char*)malloc(1024);
            snprintf(msgbuf, 1024, "cwipc_capturer: incorrect apiVersion 0x%08" PRIx64 " expected 0x%08" PRIx64 "..0x%08" PRIx64 "", apiVersion, CWIPC_API_VERSION_OLD, CWIPC_API_VERSION);
            *errorMessage = msgbuf;
        }
		return NULL;
	}
    if (strcmp(configFilename, "auto") == 0) {
        // Find any capturer that corresponds to a physical device (i.e. has a count function) that is available
        struct capturer* candidate = nullptr;
        for (auto& c : all_capturers) {
            if (c.countFunc != nullptr) {
                if (c.countFunc()) {
                    // This is a candidate. Check that there are no others.
                    if (candidate == nullptr) {
                        candidate = &c;
                    }
                    else {
                        if (errorMessage) {
                            char* msgbuf = (char*)malloc(1024);
                            snprintf(msgbuf, 1024, "cwipc_capturer: auto: cameras of multiple types found");
                            *errorMessage = msgbuf;
                        }
                        return nullptr;
                    }
                }
            }
        }
        if (candidate == nullptr) {
            if (errorMessage) {
                char* msgbuf = (char*)malloc(1024);
                snprintf(msgbuf, 1024, "cwipc_capturer: auto: no supported cameras found");
                *errorMessage = msgbuf;
            }
            return nullptr;
        }
        return candidate->factoryFunc(configFilename, errorMessage, apiVersion);
    }
    // Assure we are passed a json cameraconfig
    if (configFilename == nullptr) {
        configFilename = "cameraconfig.json";
    }
    std::string cameraType;
    json json_data;
    if (configFilename[0] == '{') {
        // Special case: a string starting with { is considered a JSON literal
        
        try {
            json_data = json::parse(configFilename);
        }
        catch (const std::exception& e) {
            if (errorMessage) {
                char* msgbuf = (char*)malloc(1024);
                snprintf(msgbuf, 1024, "cwipc_capturer: auto: JSON parse error");
                *errorMessage = msgbuf;
            }
            return nullptr;
        }
    }
    // Otherwise we check the extension. It can be .xml or .json.
    const char* extension = strrchr(configFilename, '.');
    if (strcmp(extension, ".json") == 0) {
        try {
            std::ifstream f(configFilename);
            if (!f.is_open()) {
                if (errorMessage) {
                    char* msgbuf = (char*)malloc(1024);
                    snprintf(msgbuf, 1024, "cwipc_capturer: auto: cannot open \"%s\"", configFilename);
                    *errorMessage = msgbuf;
                }
                return nullptr;
            }
            json_data = json::parse(f);

        }
        catch (const std::exception& e) {
            if (errorMessage) {
                char* msgbuf = (char*)malloc(1024);
                snprintf(msgbuf, 1024, "cwipc_capturer: auto: JSON parse error");
                *errorMessage = msgbuf;
            }
            return nullptr;
        }
    }
    std::string type;
    json_data.at("type").get_to(type);

    char* msgbuf = (char*)malloc(1024);
    snprintf(msgbuf, 1024, "cwipc_capturer: auto: cannot determine camera type from \"%s\"", configFilename);
    *errorMessage = msgbuf;
    return nullptr;
}

int _cwipc_register_capturer(const char *name, _cwipc_functype_count_devices *countFunc, _cwipc_func_capturer_factory *factoryFunc) {
    fprintf(stderr, "xxxjack _cwipc_register_capturer(%s, ...)\n", name);
    return 0;
}
