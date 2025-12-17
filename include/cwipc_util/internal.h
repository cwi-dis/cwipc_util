#ifndef _cwipc_util_internal_h_
#define _cwipc_util_internal_h_

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include <thread>

#include <nlohmann/json.hpp>

//
// This file contains definitions of types and functions that are not user-visible,
// but that are provided by cwipc_util for use by other modules.
//

#ifdef _WIN32
#include <Windows.h>

inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {
    HANDLE threadHandle = static_cast<HANDLE>(thr->native_handle());
    SetThreadDescription(threadHandle, name);
}

#else
inline void _cwipc_setThreadName(std::thread* thr, const wchar_t* name) {}
#endif

extern "C" {
    enum cwipc_log_level { LOG_ERROR, LOG_WARNING, LOG_TRACE, LOG_DEBUG };
    _CWIPC_UTIL_EXPORT void cwipc_log(cwipc_log_level level, std::string module, std::string message);
};


using json = nlohmann::json;

#define _CWIPC_CONFIG_JSON_GET(jsonobj, name, config, attr) if (jsonobj.contains(#name)) jsonobj.at(#name).get_to(config.attr)
#define _CWIPC_CONFIG_JSON_PUT(jsonobj, name, config, attr) jsonobj[#name] = config.attr

struct CwipcBaseCameraConfig {
    std::string type;

    virtual void _from_json(const json& json_data) {
        json_data.at("type").get_to(type);
    }
    virtual void _to_json(json& json_data) {
        json_data["type"] = type;
    }
};

struct CwipcBaseCaptureConfig {
    std::string type;

    virtual std::string to_string() = 0;
    virtual bool from_string(const char* buffer, std::string typeWanted) = 0;
    virtual bool from_file(const char* filename, std::string typeWanted) = 0;
    virtual void _from_json(const json& json_data) {
        json_data.at("type").get_to(type);
    }
    virtual void _to_json(json& json_data) {
        json_data["type"] = type;
        json_data["version"] = 4;
    }
};

class CwipcBaseCamera {
protected:
    std::string type;
};

class CwipcBaseCapture {
protected:
    std::string type;
};

class cwipc_tiledsource;

typedef int _cwipc_functype_count_devices();
typedef cwipc_tiledsource* _cwipc_func_capturer_factory(const char *configFilename, char **errorMessage, uint64_t apiVersion);

extern "C" {
    _CWIPC_UTIL_EXPORT int _cwipc_register_capturer(const char* name, _cwipc_functype_count_devices* countFunc, _cwipc_func_capturer_factory* factoryFunc);
}
#endif // _cwipc_util_internal_h_
