#ifndef _cwipc_util_api_h_
#define _cwipc_util_api_h_

#include <stdint.h>
#include <stdbool.h>

// For Windows ensure that the symbols are imported from a DLL, unless we're compiling the DLL itself.
#ifndef _CWIPC_UTIL_EXPORT
#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllimport)
#else
#define _CWIPC_UTIL_EXPORT
#endif
#endif

/** \brief Single point structure.
 * 
 * All data pertaining to a single point (in the external representation).
 *
 */
struct cwipc_point {
    float x;	/**< coordinate */
    float y;	/**< coordinate */
    float z;	/**< coordinate */
    uint8_t r;	/**< color */
    uint8_t g;	/**< color */
    uint8_t b;	/**< color */
};
/** \brief Version of cwipc_point structure.
 *
 * The external representation of a single point may change between versions of
 * this library. Therefore when obtaining an external representation you should pass
 * in this constant to ensure that the data is not delivered in an incorrect form.
 */
#define CWIPC_POINT_VERSION 0x20190209

#ifdef __cplusplus

#ifndef _CWIPC_PCL_POINTCLOUD_DEFINED
typedef void* cwipc_pcl_pointcloud;
#define _CWIPC_PCL_POINTCLOUD_PLACEHOLDER_DEFINED
#endif //_CWIPC_PCL_POINTCLOUD_DEFINED

/** \brief Abstract interface to a single pointcloud.
 *
 * This structure wraps a single pointcloud, and is suitable for passing around
 * without caring about the details of point data (and without having to include
 * all of the PCL headers and such). The object can be passed between different
 * shared libraries, different languages, etc.
 *
 * When used from C (as opposed to C++) this is an opaque struct _cwipc pointer.
 */ 
class cwipc {
public:
    virtual ~cwipc() {};
    
    /** \brief Deallocate the pointcloud data.
     *
     * The internal implementation of the pointcloud data may be refcounted, but
     * an explicit free method must be called to ensure the refcounting does not
     * inadvertantly free the pointcloud when dereferenced from a different DLL or
     * implementation language.
     *
     * Whoever created the cwipc object in the first place is responsible for
     * calling free, and the correct DLL will be invoked to actually free the data.
     */
    virtual void free() = 0;
    
    /** \brief Time this pointcloud was captured.
     * \return Time in microseconds, since some unspecified origin.
     */
    virtual uint64_t timestamp() = 0;
    
    /** \brief Returns size (in bytes) an external representation of this pointcloud needs.
     * \param dataVersion The type of cwipc_point data you want. Pass in CWIPC_POINT_VERSION
     *   to ensure the data is compatible with your code.
     * \return The number of bytes needed (or zero in case the format does not match
     *   or no points are available).
     */
    virtual size_t get_uncompressed_size(uint32_t dataVersion) = 0;
    
	/** \brief Get points from pointcloud in external representation format.
	 * \param pc A databuffer pointer.
	 * \param size The size of the databuffer (in bytes).
	 * \return The number of points.
	 *
	 * The caller is responsible for first calling get_uncompressed_size() and
	 * then allocating the data. This is to ensure that the memory is allocated in 
	 * a way that is compatible with the caller (for example if the caller is
	 * implemented in another language like C# or Python and needs a special
	 * allocator).
	 */
    virtual int copy_uncompressed(struct cwipc_point *pc, size_t size) = 0;
    
    /** \brief Access PCL pointcloud.
     * \return A reference to the PCL pointcloud.
     *
     * Note that this function returns a borrowed method, it is not guaranteed
     * that the reference remains valid after free() is called.
     */
    virtual cwipc_pcl_pointcloud access_pcl_pointcloud() = 0;
};

/** \brief A generator of pointclouds, abstract C++ interface.
 *
 * This interface is provided by capturers and decoders and such. It allows the
 * user of this interface to get cwipc pointcloud data.
 */
class cwipc_source {
public:
    virtual ~cwipc_source() {};
    
    /** \brief Deallocate the pointcloud source.
     *
     * Because the pointcloud source may be used in a different implementation
     * language or DLL than where it is implemented we do not count on refcounting
     * and such. Call this method if you no longer need the source.
     */
    virtual void free() = 0;
    
    /** \brief Return true if no more pointclouds are forthcoming.
     */
    virtual bool eof() = 0;
    
    /** \brief Return true if a pointcloud is currently available.
     * \param wait Set to true if the caller is willing to wait until a pointcloud is available.
     * 
     * If this cwipc_source is not multi-threading capable the wait parameter is ignored.
     * If it is multi-threaded aware and no pointcloud is currently available 
     * it may wait a reasonable amount of time (think: about a second) to see whether
     * one becomes available.
     */
    virtual bool available(bool wait) = 0;
    
    /** \brief Get a new pointcloud.
     * \return The new pointcloud.
     */
    virtual cwipc* get() = 0;
};
#else

/** \brief Abstract interface to a single pointcloud, C-compatible placeholder.
 */
typedef struct _cwipc {
	int _dummy;
} cwipc;

/** \brief C placeholder for a PCL pointcloud reference.
 */
typedef struct _cwipc_pcl_pointcloud {
	int _dummy;
} *cwipc_pcl_pointcloud;

/** \brief Abstract interface to a cwipc pointcloud source. C-compatible placeholder.
 */
typedef struct _cwipc_source {
    int _dummy;
} cwipc_source;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Read pointcloud from .ply file.
 * \param filename The ply file to read.
 * \param timestamp The timestamp to record in the cwipc object.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_read(const char *filename, uint64_t timestamp, char **errorMessage);

/** \brief Write pointcloud to .ply file.
 * \param filename The ply file to frite.
 * \param pc The pointcloud to write.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return 0 on success, -1 on failure.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT int cwipc_write(const char *filename, cwipc *pc, char **errorMessage);

/** \brief Create cwipc pointcloud from external representation.
 * \param points Pointer to buffer with points.
 * \param size Size of points in bytes.
 * \param npoint Number of points (must match size).
 * \param timestamp The timestamp to record in the cwipc object.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */ 
_CWIPC_UTIL_EXPORT cwipc *cwipc_from_points(struct cwipc_point* points, size_t size, int npoint, uint64_t timestamp, char **errorMessage);

/** \brief Read pointcloud from pointclouddump file.
 * \param filename The dump file to read.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * The dump file format is unspecified and machine dependent. It is mainly
 * intended for testing purposes.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_read_debugdump(const char *filename, char **errorMessage);

/** \brief Write pointcloud to pointclouddump file.
 * \param filename The dump file to frite.
 * \param pc The pointcloud to write.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \return 0 on success, -1 on failure.
 *
 * The dump file format is unspecified and machine dependent. It is mainly
 * intended for testing purposes.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT int cwipc_write_debugdump(const char *filename, cwipc *pc, char **errorMessage);


/** \brief Deallocate the pointcloud data (C interface).
 * \param pc The cwipc object.
 *
 * The internal implementation of the pointcloud data may be refcounted, but
 * an explicit free method must be called to ensure the refcounting does not
 * inadvertantly free the pointcloud when dereferenced from a different DLL or
 * implementation language.
 *
 * Whoever created the cwipc object in the first place is responsible for
 * calling free, and the correct DLL will be invoked to actually free the data.
 */
_CWIPC_UTIL_EXPORT void cwipc_free(cwipc *pc);

/** \brief Time this pointcloud was captured (C interface).
 * \param pc The cwipc object.
 * \return Time in microseconds, since some unspecified origin.
 */
_CWIPC_UTIL_EXPORT uint64_t cwipc_timestamp(cwipc *pc);

/** \brief Returns size (in bytes) an external representation of this pointcloud needs (C interface).
 * \param pc The cwipc object.
 * \param dataVersion The type of cwipc_point data you want. Pass in CWIPC_POINT_VERSION
 *   to ensure the data is compatible with your code.
 * \return The number of bytes needed (or zero in case the format does not match
 *   or no points are available).
 */
_CWIPC_UTIL_EXPORT size_t cwipc_get_uncompressed_size(cwipc *pc, uint32_t dataVersion);

/** \brief Get points from pointcloud in external representation format (C interface).
 * \param pc The cwipc object.
 * \param pc A databuffer pointer.
 * \param size The size of the databuffer (in bytes).
 * \return The number of points.
 *
 * The caller is responsible for first calling get_uncompressed_size() and
 * then allocating the data. This is to ensure that the memory is allocated in 
 * a way that is compatible with the caller (for example if the caller is
 * implemented in another language like C# or Python and needs a special
 * allocator).
 */
_CWIPC_UTIL_EXPORT int cwipc_copy_uncompressed(cwipc *pc, struct cwipc_point *, size_t size);

/** \brief Deallocate the pointcloud source (C interface).
 * \param src The cwipc_source object.
 *
 * Because the pointcloud source may be used in a different implementation
 * language or DLL than where it is implemented we do not count on refcounting
 * and such. Call this method if you no longer need the source.
 */
_CWIPC_UTIL_EXPORT cwipc* cwipc_source_get(cwipc_source *src);

/** \brief Get a new pointcloud (C interface).
 * \param src The cwipc_source object.
 * \return The new pointcloud.
 */
_CWIPC_UTIL_EXPORT void cwipc_source_free(cwipc_source *src);

/** \brief Return true if no more pointclouds are forthcoming.
 * \param src The cwipc_source object.
 */
_CWIPC_UTIL_EXPORT bool cwipc_source_eof(cwipc_source *src);

/** \brief Return true if a pointcloud is currently available.
 * \param src The cwipc_source object.
 * \param wait Set to true if the caller is willing to wait until a pointcloud is available.
 * 
 * If this cwipc_source is not multi-threading capable the wait parameter is ignored.
 * If it is multi-threaded aware and no pointcloud is currently available 
 * it may wait a reasonable amount of time (think: about a second) to see whether
 * one becomes available.
 */
_CWIPC_UTIL_EXPORT bool cwipc_source_available(cwipc_source *src, bool wait);

/** \brief Generate synthetic pointclouds.
 *
 * This function returns a cwipc_source that generates a rotating pointcloud
 * of the object now usually called the "water melon". It is intended for testing
 * purposes.
 */
_CWIPC_UTIL_EXPORT cwipc_source *cwipc_synthetic();

#ifdef __cplusplus
}
#endif

#endif // _cwipc_util_api_h_
