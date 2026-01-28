#include <cstddef>
#include <stdio.h>
#include <inttypes.h>
#include <chrono>
#include <iostream>
#include <sstream>

#include <cwipc_util/internal/logging.hpp>

inline std::string _level_to_string(cwipc_log_level level) {
    switch (level) {
        case CWIPC_LOG_LEVEL_ERROR: return "Error";
        case CWIPC_LOG_LEVEL_WARNING: return "Warning";
        case CWIPC_LOG_LEVEL_TRACE: return "Trace";
        case CWIPC_LOG_LEVEL_DEBUG: return "Debug";
    }
    return "Unknown-level";
}

static char **currentErrorBuf = nullptr;

void cwipc_log(cwipc_log_level level, std::string module, std::string message) {
    // No filtering on level yet, no callbacks. All that is future work.
    std::stringstream msgstream;
    msgstream << module << ": " << _level_to_string(level) << ": " << message << std::endl;
    // Output to stdout.
#if 1
    static time_t starttime = 0;
    if (starttime == 0) {
        starttime = time(0);
    }
    time_t timestamp = time(0) - starttime;
    std::cout << std::to_string(timestamp) << ": ";
#endif
    std::cout << msgstream.str();
    // xxxjack to be done: use in the callback.
    // And put in the errorbuf, if this is an error, and if there is an error buffer.
    // But don't overwrite an earlier error in the error buffer.
    if (currentErrorBuf && level == CWIPC_LOG_LEVEL_ERROR && *currentErrorBuf == nullptr) {
        // This leaks, but it shouldn't happen often...
        *currentErrorBuf = strdup(msgstream.str().c_str());
    }
}

void cwipc_log_set_errorbuf(char **errorBuf) {
    currentErrorBuf = errorBuf;
}