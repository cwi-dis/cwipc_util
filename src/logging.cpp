#include <cstddef>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>

#include <cwipc_util/internal/logging.hpp>

static char **currentErrorBuf = nullptr; //<! Pointer to pointer to error buffer string, used during calls that have an error return string.
static cwipc_log_level currentLogLevel = CWIPC_LOG_LEVEL_WARNING;
static bool initialized = false;    //<! Whether logging has been initialized.
static std::ostream* logStream(nullptr); //<! if logger has been set from the CWIPC_LOGGING env var, this is the stream to log to.
static bool log_to_callback = false; //<! Whether to log to the callback function.
static bool log_to_stderr = true; //<! Whether to log to stderr.
static bool log_to_file = false; //<! Whether to log to file.

inline std::string _level_to_string(cwipc_log_level level) {
    switch (level) {
        case CWIPC_LOG_LEVEL_ERROR: return "Error";
        case CWIPC_LOG_LEVEL_WARNING: return "Warning";
        case CWIPC_LOG_LEVEL_TRACE: return "Trace";
        case CWIPC_LOG_LEVEL_DEBUG: return "Debug";
        default: break;
    }
    return "Unknown-level";
}

static cwipc_log_level _string_to_level(const std::string &levelstr) {
    if (levelstr == "NONE") return CWIPC_LOG_LEVEL_NONE;
    if (levelstr == "ERROR") return CWIPC_LOG_LEVEL_ERROR;
    if (levelstr == "WARNING") return CWIPC_LOG_LEVEL_WARNING;
    if (levelstr == "TRACE") return CWIPC_LOG_LEVEL_TRACE;
    if (levelstr == "DEBUG") return CWIPC_LOG_LEVEL_DEBUG;
    return CWIPC_LOG_LEVEL_WARNING; // Default
}

static void initialize() {
    if (initialized) return;
    const char *env = getenv("CWIPC_LOGGING");
    if (env) {
        std::string env_s(env);
        std::string level_s;
        std::string filename_s;
        size_t sep = env_s.find(':');
        if (sep != std::string::npos) {
            level_s = env_s.substr(0, sep);
            filename_s = env_s.substr(sep + 1);
        } else {
            level_s = env_s;
        }
        currentLogLevel = _string_to_level(level_s);
        if (!filename_s.empty()) {
            logStream = new std::ofstream(filename_s, std::ios::out | std::ios::app);
        } else {
            logStream = &std::cerr;
        }
        log_to_stderr = false;
        log_to_file = true;
    }
    initialized = true;
}

void cwipc_log(cwipc_log_level level, std::string module, std::string message) {
    initialize();
    // No filtering on level yet, no callbacks. All that is future work.
    std::stringstream msgstream;
    msgstream << module << ": " << _level_to_string(level) << ": " << message << std::endl;
    // Output to stdout.

    static time_t starttime = 0;
    if (starttime == 0) {
        starttime = time(0);
    }
    time_t timestamp = time(0) - starttime;
    std::string timestamp_s("t=" + std::to_string(timestamp) + ": ");

    std::string full_message = msgstream.str();
    // Put in the errorbuf, if this is an error, and if there is an error buffer.
    // But don't overwrite an earlier error in the error buffer.
    if (currentErrorBuf && level == CWIPC_LOG_LEVEL_ERROR && *currentErrorBuf == nullptr) {
        // This leaks, but it shouldn't happen often...
        *currentErrorBuf = strdup(full_message.c_str());
    }
    if (level > currentLogLevel) {
        return;
    }
    if (log_to_stderr)
    {
        std::cerr << timestamp_s << full_message << std::endl;
    }
    if (log_to_file && logStream)
    {
        (*logStream) << timestamp_s << full_message << std::endl;
        logStream->flush();
    }
    // xxxjack send to callback.
}

void cwipc_log_set_errorbuf(char **errorBuf) {
    initialize();
    currentErrorBuf = errorBuf;
}

cwipc_log_level cwipc_log_get_level() {
    initialize();
    return currentLogLevel;
}
