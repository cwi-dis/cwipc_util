#ifndef _cwipc_util_internal_h_
#define _cwipc_util_internal_h_

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"
#include <thread>
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
struct CwipcBaseCameraConfig {
    std::string type;
};

struct CwipcBaseCaptureConfig {
    std::string type;
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
