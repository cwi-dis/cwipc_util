import ctypes
import ctypes.util
import warnings
import os

__all__ = [
    'CWIPC_API_VERSION',
    'CWIPC_POINT_PACKETHEADER_MAGIC',
    'CWIPC_FLAGS_BINARY',
    'CwipcError',
    '_cwipc_dll_search_path_collection',
    
    'cwipc',
    'cwipc_source',
    
    'cwipc_point',
    'cwipc_point_array',
    
    'cwipc_point_packetheader',
    
    'cwipc_read',
    'cwipc_read_debugdump',
    'cwipc_write',
    'cwipc_write_debugdump',
    'cwipc_from_points',
    'cwipc_from_packet',
    'cwipc_from_certh',
    
    'cwipc_synthetic',
    'cwipc_window',
    'cwipc_proxy',
    
    'cwipc_downsample',
    'cwipc_remove_outliers',
    'cwipc_tilefilter',
    'cwipc_tilemap',
    'cwipc_colormap',
    'cwipc_join',
    'cwipc_crop',
]

CWIPC_API_VERSION = 0x20220126

#
# This is a workaround for the change in DLL loading semantics on Windows since Python 3.8
# Python no longer uses the PATH environment variable to load dependent dlls but only
# its own set. For that reason we list here a set of dependencies that we know are needed,
# find those on PATH, and add the directories where those DLLs are located while loading our
# DLL.
# The list does not have to be complete, as long as at least one DLL from each directory needed
# is listed.
# NOTE: this list must be kept up-to-date otherwise loading DLLs will fail with
# an obscure message "Python could not find module .... or one of its dependencies"
#
_WINDOWS_NEEDED_DLLS=[
    "pcl_common",
    "vtkCommonCore-8.2",
    "OpenNI2",
]

class _cwipc_dll_search_path_collection:
    """Hack to ensure the correct DLL search path is used when loading a DLL on Windows"""
    def __init__(self, dlls=None):
        self.open_dll_dirs = []
        if not hasattr(os, 'add_dll_directory'):
            # Apparently we don't need to do this...
            return
        if dlls:
            # We implicitly add DLLs needed by cwipc_util
            path_entries = self._get_dll_directories(dlls + _WINDOWS_NEEDED_DLLS)
        else:
            path_string = os.environ.get('PATH')
            path_entries = path_string.split(os.pathsep)
        for p in path_entries:
            try:
                self.open_dll_dirs.append(os.add_dll_directory(p))
            except FileNotFoundError as e:
                warnings.warn(f'cwipc_dll_search_path_collection: {e}')
    
    def _get_dll_directories(self, dlls):
        done = []
        rv = []
        for dll in dlls:
            if dll in done:
                continue
            path = ctypes.util.find_library(dll)
            if not path:
                warnings.warn(f'_cwipc_dll_search_path_collection: DLL {dll} not found')
                continue
            dirname = os.path.dirname(path)
            rv.append(dirname)
            done.append(dll)
        return rv
    
    def __enter__(self):
        return self
        
    def __exit__(self, type, value, traceback):
        for d in self.open_dll_dirs:
            d.close()
            
class CwipcError(RuntimeError):
    pass
    
_cwipc_util_dll_reference = None

class cwipc_p(ctypes.c_void_p):
    pass
    
class cwipc_source_p(ctypes.c_void_p):
    pass

class cwipc_tiledsource_p(cwipc_source_p):
    pass

class cwipc_sink_p(ctypes.c_void_p):
    pass

class cwipc_auxiliary_data_p(ctypes.c_void_p):
    pass

#
# C/Python cwipc_point structure. MUST match cwipc_util/api.h, but CWIPC_API_VERSION helps a bit.
#
class cwipc_point(ctypes.Structure):
    """Point in a pointcloud. Fields ar x,y,z (float coordinates) r, g, b (color values 0..255) tile (8 bit number)"""
    _fields_ = [
        ("x", ctypes.c_float),
        ("y", ctypes.c_float),
        ("z", ctypes.c_float),
        ("r", ctypes.c_ubyte),
        ("g", ctypes.c_ubyte),
        ("b", ctypes.c_ubyte),
        ("tile", ctypes.c_ubyte),
    ]
    
    def __eq__(self, other):
        if not isinstance(other, cwipc_point):
            return False
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return False
        return True

    def __ne__(self, other):
        if not isinstance(other, cwipc_point):
            return True
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return True
        return False

#
# C/Python cwipc_vector (x/y/z).
#
class cwipc_vector(ctypes.Structure):
    """A vector"""
    _fields_ = [
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
        ("z", ctypes.c_double),
    ]
    
    def __eq__(self, other):
        if not isinstance(other, cwipc_vector):
            return False
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return False
        return True

    def __ne__(self, other):
        if not isinstance(other, cwipc_vector):
            return True
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return True
        return False
            
#
# C/Python cwipc_tileinfo structure. MUST match cwipc_util/api.h
#
class cwipc_tileinfo(ctypes.Structure):
    """Direction of a pointcloud tile. Fields are vector pointing in the direction the tile is pointing (relative to (0,0,0))"""
    _fields_ = [
        ("normal", cwipc_vector),
        ("cameraName", ctypes.c_char_p),
        ("ncamera", ctypes.c_uint8),
        ("cameraMask", ctypes.c_uint8),
    ]

#
# C/Python cwipc_point_packetheader structure
#
class cwipc_point_packetheader(ctypes.Structure):
    """Packet header for talking to cwipc_proxy server"""
    _fields_ = [
        ("hdr", ctypes.c_uint32),
        ("magic", ctypes.c_uint32),
        ("cellsize", ctypes.c_float),
        ("timestamp", ctypes.c_uint64),
        ("unused", ctypes.c_uint32),
        ("dataCount", ctypes.c_uint32),
    ]
    
CWIPC_POINT_PACKETHEADER_MAGIC = 0x20210208

CWIPC_FLAGS_BINARY = 1
#
# NOTE: the signatures here must match those in cwipc_util/api.h or all hell will break loose
#
def _cwipc_util_dll(libname=None):
    """Load the cwipc_util DLL and assign the signatures (if not already loaded)"""
    global _cwipc_util_dll_reference
    if _cwipc_util_dll_reference: return _cwipc_util_dll_reference
    
    if libname == None:
        libname = ctypes.util.find_library('cwipc_util')
        if not libname:
            raise RuntimeError('Dynamic library cwipc_util not found')
    assert libname
    with _cwipc_dll_search_path_collection(_WINDOWS_NEEDED_DLLS):
        _cwipc_util_dll_reference = ctypes.CDLL(libname)
    
    _cwipc_util_dll_reference.cwipc_read.argtypes = [ctypes.c_char_p, ctypes.c_ulonglong, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_read.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_write_ext.argtypes = [ctypes.c_char_p, cwipc_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p)]
    _cwipc_util_dll_reference.cwipc_write_ext.restype = int
    
    _cwipc_util_dll_reference.cwipc_from_points.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_ulonglong, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_from_points.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_from_packet.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_from_packet.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_from_certh.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulonglong, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_from_certh.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_read_debugdump.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_read_debugdump.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_write_debugdump.argtypes = [ctypes.c_char_p, cwipc_p, ctypes.POINTER(ctypes.c_char_p)]
    _cwipc_util_dll_reference.cwipc_write_debugdump.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_free.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_free.restype = None
    
    _cwipc_util_dll_reference.cwipc_timestamp.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_timestamp.restype = ctypes.c_ulonglong
    
    _cwipc_util_dll_reference.cwipc_cellsize.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_cellsize.restype = ctypes.c_float
    
    _cwipc_util_dll_reference.cwipc__set_cellsize.argtypes = [cwipc_p, ctypes.c_float]
    _cwipc_util_dll_reference.cwipc__set_cellsize.restype = None
    
    _cwipc_util_dll_reference.cwipc__set_timestamp.argtypes = [cwipc_p, ctypes.c_ulonglong]
    _cwipc_util_dll_reference.cwipc__set_timestamp.restype = None
    
    _cwipc_util_dll_reference.cwipc_count.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_count.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_get_uncompressed_size.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_get_uncompressed_size.restype = ctypes.c_size_t
    
    _cwipc_util_dll_reference.cwipc_copy_uncompressed.argtypes = [cwipc_p, ctypes.POINTER(ctypes.c_byte), ctypes.c_size_t]
    _cwipc_util_dll_reference.cwipc_copy_uncompressed.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_copy_packet.argtypes = [cwipc_p, ctypes.POINTER(ctypes.c_byte), ctypes.c_size_t]
    _cwipc_util_dll_reference.cwipc_copy_packet.restype = ctypes.c_size_t
    
    _cwipc_util_dll_reference.cwipc_access_auxiliary_data.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_access_auxiliary_data.restype = cwipc_auxiliary_data_p
    
    _cwipc_util_dll_reference.cwipc_source_get.argtypes = [cwipc_source_p]
    _cwipc_util_dll_reference.cwipc_source_get.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_source_available.argtypes = [cwipc_source_p, ctypes.c_bool]
    _cwipc_util_dll_reference.cwipc_source_available.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_source_eof.argtypes = [cwipc_source_p]
    _cwipc_util_dll_reference.cwipc_source_eof.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_source_free.argtypes = [cwipc_source_p]
    _cwipc_util_dll_reference.cwipc_source_free.restype = None
    
    _cwipc_util_dll_reference.cwipc_source_request_auxiliary_data.argtypes = [cwipc_source_p, ctypes.c_char_p]
    _cwipc_util_dll_reference.cwipc_source_request_auxiliary_data.restype = None
    
    _cwipc_util_dll_reference.cwipc_source_auxiliary_data_requested.argtypes = [cwipc_source_p, ctypes.c_char_p]
    _cwipc_util_dll_reference.cwipc_source_auxiliary_data_requested.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_tiledsource_seek.argtypes = [cwipc_tiledsource_p, ctypes.c_uint64]
    _cwipc_util_dll_reference.cwipc_tiledsource_seek.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_tiledsource_maxtile.argtypes = [cwipc_tiledsource_p]
    _cwipc_util_dll_reference.cwipc_tiledsource_maxtile.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_tiledsource_get_tileinfo.argtypes = [cwipc_tiledsource_p, ctypes.c_int, ctypes.POINTER(cwipc_tileinfo)]
    _cwipc_util_dll_reference.cwipc_tiledsource_get_tileinfo.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_sink_free.argtypes = [cwipc_sink_p]
    _cwipc_util_dll_reference.cwipc_sink_free.restype = None
    
    _cwipc_util_dll_reference.cwipc_sink_feed.argtypes = [cwipc_sink_p, cwipc_p, ctypes.c_bool]
    _cwipc_util_dll_reference.cwipc_sink_feed.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_sink_caption.argtypes = [cwipc_sink_p, ctypes.c_char_p]
    _cwipc_util_dll_reference.cwipc_sink_caption.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_sink_interact.argtypes = [cwipc_sink_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_int32]
    _cwipc_util_dll_reference.cwipc_sink_interact.restype = ctypes.c_char
    
    _cwipc_util_dll_reference.cwipc_synthetic.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_synthetic.restype = cwipc_tiledsource_p

    _cwipc_util_dll_reference.cwipc_window.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_window.restype = cwipc_sink_p

    _cwipc_util_dll_reference.cwipc_downsample.argtypes = [cwipc_p, ctypes.c_float]
    _cwipc_util_dll_reference.cwipc_downsample.restype = cwipc_p

    _cwipc_util_dll_reference.cwipc_remove_outliers.argtypes = [cwipc_p, ctypes.c_int, ctypes.c_float, ctypes.c_bool]
    _cwipc_util_dll_reference.cwipc_remove_outliers.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_tilefilter.argtypes = [cwipc_p, ctypes.c_int]
    _cwipc_util_dll_reference.cwipc_tilefilter.restype = cwipc_p

    _cwipc_util_dll_reference.cwipc_tilemap.argtypes = [cwipc_p, ctypes.c_char_p]
    _cwipc_util_dll_reference.cwipc_tilemap.restype = cwipc_p

    _cwipc_util_dll_reference.cwipc_colormap.argtypes = [cwipc_p, ctypes.c_ulong, ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_colormap.restype = cwipc_p

    _cwipc_util_dll_reference.cwipc_crop.argtypes = [cwipc_p, ctypes.c_float*6]
    _cwipc_util_dll_reference.cwipc_crop.restype = cwipc_p

    _cwipc_util_dll_reference.cwipc_join.argtypes = [cwipc_p, cwipc_p]
    _cwipc_util_dll_reference.cwipc_join.restype = cwipc_p

    _cwipc_util_dll_reference.cwipc_proxy.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_proxy.restype = cwipc_tiledsource_p

    _cwipc_util_dll_reference.cwipc_auxiliary_data_count.argtypes = [cwipc_auxiliary_data_p]
    _cwipc_util_dll_reference.cwipc_auxiliary_data_count.restype = ctypes.c_int

    _cwipc_util_dll_reference.cwipc_auxiliary_data_name.argtypes = [cwipc_auxiliary_data_p, ctypes.c_int]
    _cwipc_util_dll_reference.cwipc_auxiliary_data_name.restype = ctypes.c_char_p

    _cwipc_util_dll_reference.cwipc_auxiliary_data_description.argtypes = [cwipc_auxiliary_data_p, ctypes.c_int]
    _cwipc_util_dll_reference.cwipc_auxiliary_data_description.restype = ctypes.c_char_p

    _cwipc_util_dll_reference.cwipc_auxiliary_data_pointer.argtypes = [cwipc_auxiliary_data_p, ctypes.c_int]
    _cwipc_util_dll_reference.cwipc_auxiliary_data_pointer.restype = ctypes.c_void_p

    _cwipc_util_dll_reference.cwipc_auxiliary_data_size.argtypes = [cwipc_auxiliary_data_p, ctypes.c_int]
    _cwipc_util_dll_reference.cwipc_auxiliary_data_size.restype = ctypes.c_int


    return _cwipc_util_dll_reference

def cwipc_point_array(*, count=None, values=()):
    """Create an array of cwipc_point elements. `count` can be specified, or `values` can be a tuple or list of tuples (x, y, z, r, g, b, tile), or both"""
    if count == None:
        count = len(values)
    allocator = cwipc_point * count
    if isinstance(values, bytearray):
        return allocator.from_buffer(values)
    if not isinstance(values, tuple):
        values = tuple(values)
    return allocator(*values)
    
class cwipc:
    """Pointcloud as an opaque object."""
    
    def __init__(self, _cwipc=None):
        if _cwipc != None:
            assert isinstance(_cwipc, cwipc_p)
        self._cwipc = _cwipc
        self._points = None
        self._bytes = None
        
    def _as_cwipc_p(self):
        assert self._cwipc
        return self._cwipc
            
    def free(self):
        """Delete the opaque pointcloud object (by asking the original creator to do so)"""
        if self._cwipc:
            _cwipc_util_dll().cwipc_free(self._as_cwipc_p())
        self._cwipc = None
        
    def timestamp(self):
        """Returns timestamp (microseconds) when this pointcloud was captured (relative to some unspecified origin)"""
        rv = _cwipc_util_dll().cwipc_timestamp(self._as_cwipc_p())
        return rv
        
    def cellsize(self):
        """Returns the size of the cells this pointcloud represents (0 if unknown)"""
        rv = _cwipc_util_dll().cwipc_cellsize(self._as_cwipc_p())
        return rv
        
    def _set_cellsize(self, cellsize):
        """Internal use only: set the size of the cells this pointcloud represents"""
        _cwipc_util_dll().cwipc__set_cellsize(self._as_cwipc_p(), cellsize)
        
    def _set_timestamp(self, timestamp):
        """Internal use only: set the size of the cells this pointcloud represents"""
        _cwipc_util_dll().cwipc__set_timestamp(self._as_cwipc_p(), timestamp)
        
    def count(self):
        """Get the number of points in the pointcloud"""
        rv = _cwipc_util_dll().cwipc_count(self._as_cwipc_p())
        return rv
        
    def get_uncompressed_size(self):
        """Get the size in bytes of the uncompressed pointcloud data"""
        rv = _cwipc_util_dll().cwipc_get_uncompressed_size(self._as_cwipc_p())
        return rv
        
    def get_points(self):
        """Get the pointcloud data as a cwipc_point_array"""
        if self._points == None:
            self._get_points_and_bytes()
        return self._points
        
    def get_bytes(self):
        """Get the pointcloud data as Python bytes"""
        if self._bytes == None:
            self._get_points_and_bytes()
        assert self._bytes
        return self._bytes
        
    def access_auxiliary_data(self):
        rv_p = _cwipc_util_dll().cwipc_access_auxiliary_data(self._as_cwipc_p())
        if rv_p:
            return cwipc_auxiliary_data(rv_p)
        return None
        
    def _get_points_and_bytes(self):
        assert self._cwipc
        nBytes = _cwipc_util_dll().cwipc_get_uncompressed_size(self._as_cwipc_p())
        buffer = bytearray(nBytes)
        bufferCtypesType = ctypes.c_byte * nBytes
        bufferArg = bufferCtypesType.from_buffer(buffer)
        nPoints = _cwipc_util_dll().cwipc_copy_uncompressed(self._as_cwipc_p(), bufferArg, nBytes)
        points = cwipc_point_array(count=nPoints, values=buffer)
        self._bytes = buffer
        self._points = points

    def get_packet(self):
        assert self._cwipc
        nBytes = _cwipc_util_dll().cwipc_copy_packet(self._as_cwipc_p(), None, 0)
        buffer = bytearray(nBytes)
        bufferCtypesType = ctypes.c_byte * nBytes
        bufferArg = bufferCtypesType.from_buffer(buffer)
        rvNBytes = _cwipc_util_dll().cwipc_copy_packet(self._as_cwipc_p(), bufferArg, nBytes)
        assert rvNBytes == nBytes
        return buffer

class cwipc_source:
    """Pointcloud source as an opaque object"""
    
    def __init__(self, _cwipc_source=None):
        if _cwipc_source != None:
            assert isinstance(_cwipc_source, cwipc_source_p)
        self._cwipc_source = _cwipc_source

    def _as_cwipc_source_p(self):
        assert self._cwipc_source
        return self._cwipc_source
            
    def free(self):
        """Delete the opaque pointcloud source object (by asking the original creator to do so)"""
        if self._cwipc_source:
            _cwipc_util_dll().cwipc_source_free(self._as_cwipc_source_p())
        self._cwipc_source = None
        
    def eof(self):
        """Return True if no more pointclouds will be forthcoming"""
        return _cwipc_util_dll().cwipc_source_eof(self._as_cwipc_source_p())
        
    def available(self, wait):
        """Return True if a pointcloud is currently available. The wait parameter signals the source may wait a while."""
        return _cwipc_util_dll().cwipc_source_available(self._as_cwipc_source_p(), wait)
        
    def get(self):
        """Get a cwipc (opaque pointcloud) from this source. Returns None if no more pointcloudes are forthcoming"""
        rv = _cwipc_util_dll().cwipc_source_get(self._as_cwipc_source_p())
        if rv:
            return cwipc(rv)
        return None

    def request_auxiliary_data(self, name):
        """Ask this grabber to also provide auxiliary data `name` with each pointcloud"""
        if name != None: name = name.encode('utf8')
        return _cwipc_util_dll().cwipc_source_request_auxiliary_data(self._as_cwipc_source_p(), name)

    def auxiliary_data_requested(self, name):
        """Return True if this grabber provides auxiliary data `name` with each pointcloud"""
        if name != None: name = name.encode('utf8')
        return _cwipc_util_dll().cwipc_source_auxiliary_data_requested(self._as_cwipc_source_p(), name)
        
        
class cwipc_tiledsource(cwipc_source):
    """Tiled pointcloud sources as opaque object"""
    
    def __init__(self, _cwipc_tiledsource=None):
        if _cwipc_tiledsource != None:
            assert isinstance(_cwipc_tiledsource, cwipc_tiledsource_p)
        self._cwipc_source = _cwipc_tiledsource
        
    def seek(self, timestamp):
        """Return true if seek was successfull"""
        return _cwipc_util_dll().cwipc_tiledsource_seek(self._as_cwipc_source_p(),timestamp)
    
    def maxtile(self):
        """Return maximum number of tiles creatable from cwipc objects generated by this source"""
        return _cwipc_util_dll().cwipc_tiledsource_maxtile(self._as_cwipc_source_p())

    def get_tileinfo_raw(self, tilenum):
        """Return cwipc_tileinfo for tile tilenum, or None"""
        info = cwipc_tileinfo()
        rv = _cwipc_util_dll().cwipc_tiledsource_get_tileinfo(self._as_cwipc_source_p(), tilenum, ctypes.byref(info))
        if not rv:
            return None
        return info
        
    def get_tileinfo_dict(self, tilenum):
        """Return tile information for tile tilenum as Python dictionary"""
        info = self.get_tileinfo_raw(tilenum)
        if info == None:
            return info
        normal = dict(x=info.normal.x, y=info.normal.y, z=info.normal.z)
        return dict(normal=normal, cameraName=info.cameraName, ncamera=info.ncamera, cameraMask=info.cameraMask)
        
class cwipc_sink:
    """Pointcloud sink as an opaque object"""
    
    def __init__(self, _cwipc_sink=None):
        if _cwipc_sink != None:
            assert isinstance(_cwipc_sink, cwipc_sink_p)
        self._cwipc_sink = _cwipc_sink

    def _as_cwipc_sink_p(self):
        assert self._cwipc_sink
        return self._cwipc_sink
            
    def free(self):
        """Delete the opaque pointcloud sink object (by asking the original creator to do so)"""
        if self._cwipc_sink:
            _cwipc_util_dll().cwipc_sink_free(self._as_cwipc_sink_p())
        self._cwipc_source = None
        
    def feed(self, pc, clear):
        if pc != None:
            pc = pc._as_cwipc_p()
        return _cwipc_util_dll().cwipc_sink_feed(self._as_cwipc_sink_p(), pc, clear)
        
    def caption(self, caption):
        return _cwipc_util_dll().cwipc_sink_caption(self._as_cwipc_sink_p(), caption.encode('utf8'))
        
    def interact(self, prompt, responses, millis):
        if prompt != None: prompt = prompt.encode('utf8')
        if responses != None: responses = responses.encode('utf8')
        rv = _cwipc_util_dll().cwipc_sink_interact(self._as_cwipc_sink_p(), prompt, responses, millis)
        return rv.decode('utf8')
        
class cwipc_auxiliary_data:
    """Additional data attached to a cwipc object"""

    def __init__(self, _cwipc_auxiliary_data=None):
        if _cwipc_auxiliary_data != None:
            assert isinstance(_cwipc_auxiliary_data, cwipc_auxiliary_data_p)
        self._cwipc_auxiliary_data = _cwipc_auxiliary_data

    def _as_cwipc_auxiliary_data_p(self):
        assert self._cwipc_auxiliary_data
        return self._cwipc_auxiliary_data
        
    def count(self):
        return _cwipc_util_dll().cwipc_auxiliary_data_count(self._as_cwipc_auxiliary_data_p())
        
    def name(self, idx):
        rv = _cwipc_util_dll().cwipc_auxiliary_data_name(self._as_cwipc_auxiliary_data_p(), idx)
        return rv.decode('utf8')
        
    def description(self, idx):
        rv = _cwipc_util_dll().cwipc_auxiliary_data_description(self._as_cwipc_auxiliary_data_p(), idx)
        return rv.decode('utf8')
        
    def pointer(self, idx):
        return _cwipc_util_dll().cwipc_auxiliary_data_pointer(self._as_cwipc_auxiliary_data_p(), idx)
        
    def size(self, idx):
        return _cwipc_util_dll().cwipc_auxiliary_data_size(self._as_cwipc_auxiliary_data_p(), idx)
        
    def data(self, idx):
        size = self.size(idx)
        pointer = self.pointer(idx)
        c_type = ctypes.c_ubyte*size
        c_array = c_type.from_address(pointer)
        rv = bytearray(c_array)
        return rv
        
        
def cwipc_read(filename, timestamp):
    """Read pointcloud from a .ply file, return as cwipc object. Timestamp must be passsed in too."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_read(filename.encode('utf8'), timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc(rv)
    return None
    
def cwipc_write(filename, pointcloud, flags=0):
    """Write a cwipc object to a .ply file."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_write_ext(filename.encode('utf8'), pointcloud._as_cwipc_p(), flags, ctypes.byref(errorString))
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    return rv
    
def cwipc_from_points(points, timestamp):
    """Create a cwipc from either `cwipc_point_array` or a list or tuple of xyzrgb values"""
    if not isinstance(points, ctypes.Array):
        points = cwipc_point_array(values=points)
    addr = ctypes.addressof(points)
    nPoint = len(points)
    nBytes = ctypes.sizeof(points)
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_from_points(addr, nBytes, nPoint, timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc(rv)
    return None
    
def cwipc_from_packet(packet):
    nBytes = len(packet)
    byte_array_type = ctypes.c_char * nBytes
    try:
        c_packet = byte_array_type.from_buffer(packet)
    except TypeError:
        c_packet = byte_array_type.from_buffer_copy(packet)
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_from_packet(c_packet, nBytes, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc(rv)
    return None

def cwipc_from_certh(certhPC, timestamp, origin=None, bbox=None):
    """Create a cwipc from a CERTH PointCloud structure (address passed as ctypes.c_void_p)"""
    if not isinstance(certhPC, ctypes.c_void_p):
        certhPC = ctypes.cast(certhPC, ctypes.c_void_p)
    if origin:
        origin = (ctypes.c_float*3)(*origin)
        origin = ctypes.cast(origin, ctypes.c_void_p)
    if bbox:
        bbox = (ctypes.c_float*6)(*bbox)
        bbox = ctypes.cast(bbox, ctypes.c_void_p)
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_from_certh(certhPC, origin, bbox, timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc(rv)
    return None
    
def cwipc_read_debugdump(filename):
    """Return a cwipc object read from a .cwipcdump file."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_read_debugdump(filename.encode('utf8'), ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc(rv)
    return None
    
def cwipc_write_debugdump(filename, pointcloud):
    """Write a cwipc object to a .cwipcdump file."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_write_debugdump(filename.encode('utf8'), pointcloud._as_cwipc_p(), ctypes.byref(errorString))
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    return rv
    
def cwipc_synthetic(fps=0, npoints=0):
    """Returns a cwipc_source object that returns synthetically generated cwipc objects on every get() call."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_synthetic(fps, npoints, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource(rv)
    return None
    
def cwipc_window(title):
    """Returns a cwipc_sink object that displays pointclouds in a window"""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_window(title.encode('utf8'), ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_sink(rv)
    return None
    
def cwipc_downsample(pc, voxelsize):
    rv = _cwipc_util_dll().cwipc_downsample(pc._as_cwipc_p(), voxelsize)
    return cwipc(rv)
    
def cwipc_remove_outliers(pc, kNeighbors, stdDesvMultThresh, perTile):
    rv = _cwipc_util_dll().cwipc_remove_outliers(pc._as_cwipc_p(), kNeighbors, stdDesvMultThresh, perTile)
    return cwipc(rv)
    
def cwipc_tilefilter(pc, tile):
    rv = _cwipc_util_dll().cwipc_tilefilter(pc._as_cwipc_p(), tile)
    return cwipc(rv)
  
def cwipc_tilemap(pc, mapping):
    if type(mapping) != bytes and type(mapping) != bytearray:
        m = [0]*256
        for k in mapping:
            m[k] = mapping[k]
        mapping = m
    rv = _cwipc_util_dll().cwipc_tilemap(pc._as_cwipc_p(), bytes(mapping))
    return cwipc(rv)
  
def cwipc_colormap(pc, clearBits, setBits):
    rv = _cwipc_util_dll().cwipc_colormap(pc._as_cwipc_p(), clearBits, setBits)
    return cwipc(rv)
  
def cwipc_crop(pc, bbox):
    bbox_arg = (ctypes.c_float*6)(*bbox)
    rv = _cwipc_util_dll().cwipc_crop(pc._as_cwipc_p(), bbox_arg)
    return cwipc(rv)
  
def cwipc_join(pc1, pc2):
    rv = _cwipc_util_dll().cwipc_join(pc1._as_cwipc_p(), pc2._as_cwipc_p())
    return cwipc(rv)
  
def cwipc_proxy(host, port):
    """Returns a cwipc_source object that starts a server and receives pointclouds over a socket connection"""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_proxy(host.encode('utf8'), port, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource(rv)
    return None
    
  
def main():
    generator = cwipc_synthetic()
    pc = generator.get()
    cwipc_write_debugdump('output.cwipcdump', pc)
    cwipc_write('output.ply', pc)
    
if __name__ == '__main__':
    main()
    
    
