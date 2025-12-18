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
    bool disabled = false;      // to easily disable cameras without altering too much the cameraconfig
    bool connected = false;     // set to true when the camera is opened.
    std::string serial;         // Serial number of this camera
    std::string filename = "";     // filename, for playback
    pcl::shared_ptr<Eigen::Affine3d> trafo; //!< Transformation matrix from camera coorindates to world coordinates
    cwipc_vector cameraposition = { 0,0,0 };//!< Position of this camera in real world coordinates

    virtual void _from_json(const json& json_data) {
        json_data.at("type").get_to(type);
        json_data.at("serial").get_to(serial);
        json_data.at("disabled").get_to(disabled);
        if (json_data.contains("filename")) {
            json_data.at("filename").get_to(filename);
        } else if (json_data.contains("playback_filename")) {
            // backwards compatibility
            json_data.at("playback_filename").get_to(filename);
        }
        // cameraposition is not serialized, it will be re-computed from the trafo.
        if (json_data.contains("trafo")) {
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 4; y++) {
                    (*trafo)(x, y) = json_data["trafo"][x][y];
                }
            }
        }
    };

    virtual void _to_json(json& json_data) {
        json_data["type"] = type;
        json_data["serial"] = serial;
        json_data["disabled"] = disabled;
        if (filename != "") {
            json_data["filename"] = filename;
        }
        if (cameraposition.x != 0.0 || cameraposition.y != 0.0 || cameraposition.z != 0.0) {
            json_data["_cameraposition"] = {
                cameraposition.x,
                cameraposition.y,
                cameraposition.z
            };
        }
        json_data["trafo"] = {
            {(*trafo)(0, 0), (*trafo)(0, 1), (*trafo)(0, 2), (*trafo)(0, 3)},
            {(*trafo)(1, 0), (*trafo)(1, 1), (*trafo)(1, 2), (*trafo)(1, 3)},
            {(*trafo)(2, 0), (*trafo)(2, 1), (*trafo)(2, 2), (*trafo)(2, 3)},
            {(*trafo)(3, 0), (*trafo)(3, 1), (*trafo)(3, 2), (*trafo)(3, 3)}
        };
    };
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
