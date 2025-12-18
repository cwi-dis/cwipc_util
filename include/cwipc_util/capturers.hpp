#ifndef _cwipc_util_capturers_hpp_
#define _cwipc_util_capturers_hpp_

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

/** Base class for per-camera configuration
 * 
 * Stores attributes common to all camera types, plus methods to serialize and deserialize.
 */
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

/** Base class for capture configuration
 * 
 * Stores attributes common to all capturer types, plus methods to serialize and deserialize.
 */
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

/** Base class for camera 
 * 
*/
class CwipcBaseCamera {
protected:
    std::string type;
};

/** Base class for capturer 
 * 
*/
class CwipcBaseCapture {
protected:
    std::string type;
};

/** Template base class for capturer implementations
 * 
 * Two template parameters:
 * - GrabberClass: The class that actually implements the capturing
 * - CameraConfigClass: The class that implements per-camera configuration storage
 * Most implementations will have an extra layer of inheritance, to implement
 * common functionality provided by the API that is implemented both for live
 * cameras and for playback.
 */

template<class GrabberClass, class CameraConfigClass>
class cwipc_capturer_impl_base : public cwipc_tiledsource {
protected:
    GrabberClass *m_grabber; 
public:
    cwipc_capturer_impl_base(const char* configFilename) 
    : m_grabber(GrabberClass::factory())
    {
        m_grabber->config_reload(configFilename);
    }

    virtual ~cwipc_capturer_impl_base() {
        delete m_grabber;
        m_grabber = NULL;
    }

    bool is_valid() {
        return m_grabber->camera_count > 0;
    }

    void free() {
        delete m_grabber;
        m_grabber = NULL;
    }

    virtual size_t get_config(char* buffer, size_t size) override
    {
        auto config = m_grabber->config_get();

        if (buffer == nullptr) {
            return config.length();
        }

        if (size < config.length()) {
            return 0;
        }

        memcpy(buffer, config.c_str(), config.length());
        return config.length();
    }

    virtual bool reload_config(const char* configFile) override {
        return m_grabber->config_reload(configFile);
    }

    bool eof() {
        return m_grabber->eof;
    }

    bool available(bool wait) {
        if (m_grabber == NULL) {
            return false;
        }

        return m_grabber->pointcloud_available(wait);
    }

    cwipc* get() {
        if (m_grabber == NULL) {
            return NULL;
        }

        cwipc* rv = m_grabber->get_pointcloud();
        return rv;
    }

    int maxtile() {
        if (m_grabber == NULL) {
            return 0;
        }

        int nCamera = m_grabber->configuration.all_camera_configs.size();
        if (nCamera <= 1) {
            // Using a single camera or no camera.
            return nCamera;
        }

        return nCamera + 1;
    }

    bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) {
        if (m_grabber == NULL) {
            return false;
        }

        int nCamera = m_grabber->configuration.all_camera_configs.size();

        if (nCamera == 0) { // No camera
            return false;
        }

        if (tilenum < 0 || tilenum >= nCamera+1) {
            return false;
        }
        if (tilenum == 0) {
            // Special case: the whole pointcloud
            if (tileinfo) {
                tileinfo->normal = { 0, 0, 0 };
                tileinfo->cameraName = NULL;
                tileinfo->ncamera = nCamera;
                tileinfo->cameraMask = 0; // All cameras contributes to this
            }
            return true;
        }
        CameraConfigClass &cameraConfig = m_grabber->configuration.all_camera_configs[tilenum-1];
        if (tileinfo) {
            tileinfo->normal = cameraConfig.cameraposition; // Use the camera position as the normal
            tileinfo->cameraName = (char *)cameraConfig.serial.c_str();
            tileinfo->ncamera = 1; // Only one camera contributes to this
            tileinfo->cameraMask = (uint8_t)1 << (tilenum-1); // Only this camera contributes
        }
        return true;
    }
    
    virtual void request_auxiliary_data(const std::string &name) = 0;
    virtual bool auxiliary_operation(const std::string op, const void* inbuf, size_t insize, void* outbuf, size_t outsize) = 0;
    virtual bool seek(uint64_t timestamp) = 0;
};

/** Capturer registration.
 * Capturer implementations should register themselves by calling
 * _cwipc_register_capturer() during their initialization.
 */
class cwipc_tiledsource;

typedef int _cwipc_functype_count_devices();
typedef cwipc_tiledsource* _cwipc_func_capturer_factory(const char *configFilename, char **errorMessage, uint64_t apiVersion);

extern "C" {
    _CWIPC_UTIL_EXPORT int _cwipc_register_capturer(const char* name, _cwipc_functype_count_devices* countFunc, _cwipc_func_capturer_factory* factoryFunc);
}
#endif // _cwipc_util_capturers_hpp_
