#ifndef _cwipc_util_internal_logging_hpp_
#define _cwipc_util_internal_logging_hpp_
#pragma once

#include "cwipc_util/api.h"


extern "C" {
    /** Call to emit a log message.
     * Mainly meant for internal use within cwipc, so messages get forwarded to the correct recipient.
     */
    _CWIPC_UTIL_EXPORT void cwipc_log(cwipc_log_level level, std::string module, std::string message);
    /** Set error capture buffer.
     * Called internally by methods that have a char **errorMessage argument, and
     * cleared at the end of the method.
     * This will capture the most recent error here.
     */
    _CWIPC_UTIL_EXPORT void cwipc_log_set_errorbuf(char** errorbuf);
    /** Get current global log level
     *
     */
    _CWIPC_UTIL_EXPORT cwipc_log_level cwipc_log_get_level();
};


/** Base class for classes that do logging.
 *
 * Only handles logging.
 */
class CwipcLoggingBase {
protected:
    std::string CLASSNAME;  //!< For error, warning and debug messages only
    CwipcLoggingBase(std::string _CLASSNAME)
        : CLASSNAME(_CLASSNAME)
    {
    }

    inline void _log(cwipc_log_level level, std::string message) const {
        cwipc_log(level, CLASSNAME, message);
    }

public:
    inline void _log_error(std::string message) const {
        cwipc_log(CWIPC_LOG_LEVEL_ERROR, CLASSNAME, message);
    }

    inline void _log_warning(std::string message) const {
        cwipc_log(CWIPC_LOG_LEVEL_WARNING, CLASSNAME, message);
    }

    inline void _log_trace(std::string message) const {
        cwipc_log(CWIPC_LOG_LEVEL_TRACE, CLASSNAME, message);
    }

    inline void _log_debug(std::string message) const {
#ifdef CWIPC_DEBUG
        cwipc_log(CWIPC_LOG_LEVEL_DEBUG, CLASSNAME, message);
#endif
    }

    inline void _log_debug_thread(std::string message) const {
#ifdef CWIPC_DEBUG_THREAD
        cwipc_log(CWIPC_LOG_LEVEL_DEBUG, CLASSNAME + " (thread)", message);
#endif
    }
};

#endif // _cwipc_util_internal_logging_hpp_
