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

/** \brief Version of cwipc API.
 *
 * Version of the current API of cwipc. Pass to constructors to ensure library
 * compatibility.
 */
#define CWIPC_API_VERSION 0x20201022

/** \brief Version of oldest compatible cwipc API.
 *
 * Version of the oldest API of cwipc to which this set of libraries is compatible.
 */
#define CWIPC_API_VERSION_OLD 0x20200703


/** \brief Single 3D vector.
 *
 * Coordinates of a single 3D point.
 *
 */
struct cwipc_vector {
	double x;		/**< coordinate */
	double y;		/**< coordinate */
	double z;		/**< coordinate */
};

/** \brief Single point structure.
 * 
 * All data pertaining to a single point (in the external representation).
 *
 */
struct cwipc_point {
	float x;		/**< coordinate */
	float y;		/**< coordinate */
	float z;		/**< coordinate */
	uint8_t r;		/**< color */
	uint8_t g;		/**< color */
	uint8_t b;		/**< color */
	uint8_t tile;	/**< tile number (can also be interpreted as mask of contributing cameras) */
};

/** \brief header for transferring cwipc_point data over a network connection
 */
struct cwipc_point_packetheader {
    uint32_t magic; /**< magic number */
    uint32_t dataCount; /**< Number of bytes following this header */
    uint64_t timestamp; /**< Timestamp of the pointcloud */
    float cellsize; /**< Size of a single point */
    uint32_t unused;    /**< extra field to fill out structure to 24 bytes (multiple of 8) */
};

/** \brief magic number for use in cwipc_point_packetheader
 */
#define CWIPC_POINT_PACKETHEADER_MAGIC 0x20201016

/** \brief Information on a tile.
 *
 * Information on tiles with a certain tile number, a vector of length 1 indicating which way the tile is pointing.
 * Tiles that face in no particular direction have length 0.
 * The structure also has information on how many cameras contribute to this tile, and for single-camera
 * tiles a pointer to a unique ID of the camera (static string).
 */
struct cwipc_tileinfo {
	struct cwipc_vector normal;	/**< Normal indicating the direction the tile is facing */
	char *camera; 				/**< Identifier of the camera (static string) or NULL */
	uint8_t ncamera; 			/**< Number of cameras that potentially contribute to this tile */
};

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
     * \return Time in milliseconds, since some unspecified origin.
     */
    virtual uint64_t timestamp() = 0;
    
    /** \brief Returns the grid cell size at which this pointcloud was created, if known.
     * \return Either a size (in the same coordinates as x, y and z) or 0 if unknown.
     *
     * If the pointcloud is to be displayed the number returned by this call is a good
     * guess for a pointsize to use to show an obect that doesn't have any holes in it.
     */
    virtual float cellsize() = 0;
    
    /** \brief Semi-private method to initialize the cellsize. Not for general use.
	 * Passing a negative number will use a heuristic to determine a reasonable value for
	 * the cellsize.
     */
    virtual void _set_cellsize(float cellsize) = 0;
    
    /** \brief Returns the number of points in the pointcloud.
	 *  \return the point count
	 */
    virtual int count() = 0;

    /** \brief Returns size (in bytes) an external representation of this pointcloud needs.
     * \return The number of bytes needed (or zero in case the format does not match
     *   or no points are available).
     */

    virtual size_t get_uncompressed_size() = 0;
    
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

/** \brief A generator of tiled pointclouds, abstract C++ interface.
 *
 * This interface is provided by capturers and decoders and such that can
 * return pointclouds with tiling information. It is a subclass of
 * cwipc_source with extra methods to obtain information on available
 * tiles in the produced pointclouds.
 */
class cwipc_tiledsource : public cwipc_source {
public:
    virtual ~cwipc_tiledsource() {};
    
    virtual void free() = 0;
    virtual bool eof() = 0;
    virtual bool available(bool wait) = 0;
    virtual cwipc* get() = 0;
    
    /** \brief Return maximum number of possible tiles returned.
     *
     * Pointclouds produced by this source will (even after tile-processing)
     * never contain more tiles than this.
     * Note that this number may therefore be higher than the actual number of
     * tiles ever occurring in a pointcloud: a next tiling step may combine
     * points into new tiles.
     */
    virtual int maxtile() = 0;
    
    /** \brief Return information on a tile number.
     * \param tilenum The tile on which to obtain information.
     * \param tileinfo A pointer to a structure filled with information on the tile (if non-NULL).
     * \return A boolean that is true if the tile could ever exist.
     *
     * Tile number 0 is a special case, representing the whole pointcloud.
     */
    virtual bool get_tileinfo(int tilenum, struct cwipc_tileinfo *tileinfo) = 0;
};

/** \brief A consumer of pointclouds, abstract C++ interface.
 *
 * This interface is provided by renderers and such. It allows the
 * user of this interface to send cwipc pointcloud data somewhere.
 *
 */
class cwipc_sink {
public:
    virtual ~cwipc_sink() {};
    
    /** \brief Deallocate the pointcloud sink.
     *
     * Because the pointcloud sink may be used in a different implementation
     * language or DLL than where it is implemented we do not count on refcounting
     * and such. Call this method if you no longer need the source.
     */
    virtual void free() = 0;
    
    /** \brief Feed a pointcloud to the sink.
     * \param pc The pointcloud
     * \param clear If true a display window will clear any previous pointclouds
     * \return True if the operation was successful.
     *
     * A display sink will likely show the pointcloud in a window and give the
     * user some interaction commands to inspect it.
     *
     * Note that if the sink needs to keep the pointcloud data it will make a
     * copy, so after feed() returns the caller can safely call pc.free().
     */
    virtual bool feed(cwipc *pc, bool clear) = 0;

    /** \brief Set a caption or title on the window.
     * \param caption The UTF8 caption string.
     * \return True if this sink could present the caption to the user.
     *
     * If the sink is a display window this will set some sort of caption
     * on the window.
     */
    virtual bool caption(const char *caption) = 0;
    
    /** \brief User interaction.
     * \param prompt A prompt message to show to the user, explaining what the program wants.
     * \param reponses A string with all characters that can be typed by the user.
     * \param millis The number of milliseconds to wait for interaction, 0 for no wait or -1 for forever.
     * \return The character typed by the user, or '\0' if this sink does not support user interaction.
     */
    virtual char interact(const char *prompt, const char *responses, int32_t millis) = 0;
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

/** \brief Abstract interface to a cwipc tiled pointcloud source. C-compatible placeholder.
 */
typedef struct _cwipc_tiledsource {
    struct _cwipc_source source;
} cwipc_tiledsource;

/** \brief Abstract interface to a cwipc pointcloud sink. C-compatible placeholder.
 */
typedef struct _cwipc_sink {
    int _dummy;
} cwipc_sink;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \brief Read pointcloud from .ply file.
 * \param filename The ply file to read.
 * \param timestamp The timestamp to record in the cwipc object.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_read(const char *filename, uint64_t timestamp, char **errorMessage, uint64_t apiVersion);

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
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_from_points(struct cwipc_point* points, size_t size, int npoint, uint64_t timestamp, char **errorMessage, uint64_t apiVersion);

/** \brief Create cwipc pointcloud from CERTH pointcloud representation.
 * \param points Pointer to CERTH PointCloud structure.
 * \param origin Optional point to coordinates of point (3 floats x, y, z) that needs to be moved to (0, 0, 0)
 * \param bbox Optional pointer to bounding box (6 floats: minx, maxx, miny, maxy, minz, maxz)
 * \param timestamp The timestamp to record in the cwipc object.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_from_certh(void* certhPC, float *origin, float *bbox, uint64_t timestamp, char **errorMessage, uint64_t apiVersion);
    
/** \brief Read pointcloud from pointclouddump file.
 * \param filename The dump file to read.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 * \return the abstract point cloud, or NULL in case of errors.
 *
 * The dump file format is unspecified and machine dependent. It is mainly
 * intended for testing purposes.
 *
 * If an error occurs and errorMessage is non-NULL it will receive a pointer to
 * a string with the message.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_read_debugdump(const char *filename, char **errorMessage, uint64_t apiVersion);

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
 * \return Time in milliseconds, since some unspecified origin.
 */
_CWIPC_UTIL_EXPORT uint64_t cwipc_timestamp(cwipc *pc);

/** \brief Returns the grid cell size at which this pointcloud was created, if known.
 * \return Either a size (in the same coordinates as x, y and z) or 0 if unknown.
 *
 * If the pointcloud is to be displayed the number returned by this call is a good
 * guess for a pointsize to use to show an obect that doesn't have any holes in it.
 */
_CWIPC_UTIL_EXPORT float cwipc_cellsize(cwipc *pc);
    
/** \brief Semi-private method to initialize the cellsize. Not for general use.
 */
_CWIPC_UTIL_EXPORT void cwipc__set_cellsize(cwipc *pc, float cellsize);

/** \brief Returns number of points in the pointcloud.
 * \return The number of points.
 */
_CWIPC_UTIL_EXPORT int cwipc_count(cwipc *pc);

/** \brief Returns size (in bytes) an external representation of this pointcloud needs (C interface).
* \param pc The cwipc object.
* \return The number of bytes needed (or zero in case the format does not match
*   or no points are available).
*/
_CWIPC_UTIL_EXPORT size_t cwipc_get_uncompressed_size(cwipc *pc);

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

/** \brief Get a new pointcloud (C interface).
 * \param src The cwipc_source object.
 * \return The new pointcloud.
 */
_CWIPC_UTIL_EXPORT cwipc* cwipc_source_get(cwipc_source *src);

/** \brief Deallocate the pointcloud source (C interface).
 * \param src The cwipc_source object.
 *
 * Because the pointcloud source may be used in a different implementation
 * language or DLL than where it is implemented we do not count on refcounting
 * and such. Call this method if you no longer need the source.
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

/** \brief Return maximum number of possible tiles returned.
 * \param src The cwipc_tiledsource object.
 *
 * Pointclouds produced by this source will (even after tile-processing)
 * never contain more tiles than this.
 * Note that this number may therefore be higher than the actual number of
 * tiles ever occurring in a pointcloud: a next tiling step may combine
 * points into new tiles.
 */
_CWIPC_UTIL_EXPORT int cwipc_tiledsource_maxtile(cwipc_tiledsource *src);

/** \brief Return information on a tile number.
 * \param src The cwipc_source object.
 * \param tilenum The tile on which to obtain information.
 * \param tileinfo A pointer to a structure filled with information on the tile (if non-NULL).
 * \return A boolean that is true if the tile could ever exist.
 *
 * Tile number 0 is a special case, representing the whole pointcloud.
 */
_CWIPC_UTIL_EXPORT bool cwipc_tiledsource_get_tileinfo(cwipc_tiledsource *src, int tilenum, struct cwipc_tileinfo *tileinfo);

/** \brief Deallocate the pointcloud sink.
 *
 * Because the pointcloud sink may be used in a different implementation
 * language or DLL than where it is implemented we do not count on refcounting
 * and such. Call this method if you no longer need the source.
 */
_CWIPC_UTIL_EXPORT void cwipc_sink_free(cwipc_sink *sink);

/** \brief Feed a pointcloud to the sink.
 * \param pc The pointcloud
 * \param clear If true a display window will clear any previous pointclouds
 * \return True if the operation was successful.
 *
 * A display sink will likely show the pointcloud in a window and give the
 * user some interaction commands to inspect it.
 *
 * Note that if the sink needs to keep the pointcloud data it will make a
 * copy, so after feed() returns the caller can safely call pc.free().
 */
_CWIPC_UTIL_EXPORT bool cwipc_sink_feed(cwipc_sink *sink, cwipc *pc, bool clear);

/** \brief Set a caption or title on the window.
 * \param caption The UTF8 caption string.
 * \return True if this sink could present the caption to the user.
 *
 * If the sink is a display window this will set some sort of caption
 * on the window.
 */
_CWIPC_UTIL_EXPORT bool cwipc_sink_caption(cwipc_sink *sink, const char *caption);

/** \brief User interaction.
 * \param prompt A prompt message to show to the user, explaining what the program wants.
 * \param reponses A string with all characters that can be typed by the user.
 * \param millis The number of milliseconds to wait for interaction, 0 for no wait or -1 for forever.
 * \return The character typed by the user, or '\0' if the user did not type anything,
 * or if this sink does not support user interaction.
 */
_CWIPC_UTIL_EXPORT char cwipc_sink_interact(cwipc_sink *sink, const char *prompt, const char *responses, int32_t millis);
    
/** \brief Generate synthetic pointclouds.
 * \param fps Maximum frames-per-second produced (0 for unlimited)
 * \param npoints Approximate number of points in pointcloud (0 for default 160Kpoint)
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 *
 * This function returns a cwipc_source that generates a rotating pointcloud
 * of the object now usually called the "water melon". It is intended for testing
 * purposes.
 */
_CWIPC_UTIL_EXPORT cwipc_tiledsource *cwipc_synthetic(int fps, int npoints, char **errorMessage, uint64_t apiVersion);

/** \brief Display a window to show pointclouds.
 * \param Title The title string, to be shown in the title bar of the window.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 *
 * This function will open a window and returns a cwipc_sink. When the program
 * feeds pointclouds into the sink these will be displayed in the window.
 * The user has some interaction commands to inspect the pointcloud.
 *
 * Note that it may be necessary (depending on the operating system) to call this
 * function only from the main thread, and to call the methods on the cwipc_sink
 * returned also only from the main thread.
 */
_CWIPC_UTIL_EXPORT cwipc_sink *cwipc_window(const char *title, char **errorMessage, uint64_t apiVersion);


/** \brief Downsample a cwipc pointcloud.
 * \param pc The source pointcloud
 * \param voxelsize Wanted resolution for returned pointcloud.
 * \return a new pointcloud
 *
 * A grid of voxelsize*voxelsize*voxelsize is overlaid over the pointcloud. All
 * points within each cube are combined into the new point, with coordinates and colors
 * of that new point being a smart average of the original points in the cell.
 * The tile of the new point is the OR of the tiles of the contributing points.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_downsample(cwipc *pc, float voxelsize);

/** \brief Filter a pointcloud by tile.
 * \param pc The source pointcloud.
 * \param tile The tile number.
 *
 * Returns a new pointcloud (possibly empty) that only consists only of the points
 * with the given tile number.
 */
_CWIPC_UTIL_EXPORT cwipc *cwipc_tilefilter(cwipc *pc, int tile);
 
/** \brief Receive pointclouds over a socket connection.
 * \param host Local hostname or IP address to bind socket to (default: 0.0.0.0)
 * \param port Local port number to bind socket to.
 * \param errorMessage Address of a char* where any error message is saved (or NULL).
 * \param apiVersion Pass in CWIPC_API_VERSION to ensure dll compatibility.
 *
 * This function creates a server (in a separate thread) that listens on the given port
 * for an incoming pointcloud stream. Those pointclouds are then returned similar as to
 * synthetic or normal grabbed pointclouds.
 */
_CWIPC_UTIL_EXPORT cwipc_tiledsource *cwipc_proxy(const char *host, int port, char **errorMessage, uint64_t apiVersion);

#ifdef __cplusplus
}
#endif

#endif // _cwipc_util_api_h_
