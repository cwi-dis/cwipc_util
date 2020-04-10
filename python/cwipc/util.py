import ctypes
import ctypes.util

__all__ = [
    'CWIPC_API_VERSION',
    'CwipcError',
    
    'cwipc',
    'cwipc_source',
    
    'cwipc_point',
    'cwipc_point_array',
    
    'cwipc_read',
    'cwipc_read_debugdump',
    'cwipc_write',
    'cwipc_write_debugdump',
    'cwipc_from_points',
    'cwipc_from_certh',
    
    'cwipc_synthetic',
    'cwipc_window'
]

CWIPC_API_VERSION = 0x20190522

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
# C/Python cwipc_tileinfo structure. MUST match cwipc_util/api.h, but CWIPC_TILEINFO_VERSION helps a bit.
#
class cwipc_tileinfo(ctypes.Structure):
    """Direction of a pointcloud tile. Fields are vector pointing in the direction the tile is pointing (relative to (0,0,0))"""
    _fields_ = [
        ("normal", cwipc_vector),
        ("camera", ctypes.c_char_p),
        ("ncamera", ctypes.c_uint8),
    ]

CWIPC_TILEINFO_VERSION = 0x20190516

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
    _cwipc_util_dll_reference = ctypes.CDLL(libname)
    
    _cwipc_util_dll_reference.cwipc_read.argtypes = [ctypes.c_char_p, ctypes.c_ulonglong, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_read.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_write.argtypes = [ctypes.c_char_p, cwipc_p, ctypes.POINTER(ctypes.c_char_p)]
    _cwipc_util_dll_reference.cwipc_write.restype = int
    
    _cwipc_util_dll_reference.cwipc_from_points.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int, ctypes.c_ulonglong, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_from_points.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_from_certh.argtypes = [ctypes.c_void_p, ctypes.c_ulonglong, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
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
    
    _cwipc_util_dll_reference.cwipc_count.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_count.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_get_uncompressed_size.argtypes = [cwipc_p]
    _cwipc_util_dll_reference.cwipc_get_uncompressed_size.restype = ctypes.c_size_t
    
    _cwipc_util_dll_reference.cwipc_copy_uncompressed.argtypes = [cwipc_p, ctypes.POINTER(ctypes.c_byte), ctypes.c_size_t]
    _cwipc_util_dll_reference.cwipc_copy_uncompressed.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_source_get.argtypes = [cwipc_source_p]
    _cwipc_util_dll_reference.cwipc_source_get.restype = cwipc_p
    
    _cwipc_util_dll_reference.cwipc_source_available.argtypes = [cwipc_source_p, ctypes.c_bool]
    _cwipc_util_dll_reference.cwipc_source_available.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_source_eof.argtypes = [cwipc_source_p]
    _cwipc_util_dll_reference.cwipc_source_eof.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_source_free.argtypes = [cwipc_source_p]
    _cwipc_util_dll_reference.cwipc_source_free.restype = None
    
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
    
    _cwipc_util_dll_reference.cwipc_synthetic.argtypes = [ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_synthetic.restype = cwipc_tiledsource_p

    _cwipc_util_dll_reference.cwipc_window.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_window.restype = cwipc_sink_p

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
        
class cwipc_tiledsource(cwipc_source):
    """Tiled pointcloud sources as opaque object"""
    
    def __init__(self, _cwipc_tiledsource=None):
        if _cwipc_tiledsource != None:
            assert isinstance(_cwipc_tiledsource, cwipc_tiledsource_p)
        self._cwipc_source = _cwipc_tiledsource
    
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
        return dict(normal=normal, camera=info.camera, ncamera=info.ncamera)
        
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
        
def cwipc_read(filename, timestamp):
    """Read pointcloud from a .ply file, return as cwipc object. Timestamp must be passsed in too."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_read(filename.encode('utf8'), timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc(rv)
    return None
    
def cwipc_write(filename, pointcloud):
    """Write a cwipc object to a .ply file."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_write(filename.encode('utf8'), pointcloud._as_cwipc_p(), ctypes.byref(errorString))
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
    
def cwipc_from_certh(certhPC, timestamp):
    """Create a cwipc from a CERTH PointCloud structure (address passed as ctypes.c_void_p)"""
    if not isinstance(certhPC, ctypes.c_void_p):
        certhPC = ctypes.cast(certhPC, ctypes.c_void_p)
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_from_certh(certhPC, timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
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
    
def cwipc_synthetic():
    """Returns a cwipc_source object that returns synthetically generated cwipc objects on every get() call."""
    errorString = ctypes.c_char_p()
    rv = _cwipc_util_dll().cwipc_synthetic(ctypes.byref(errorString), CWIPC_API_VERSION)
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
    
def main():
    generator = cwipc_synthetic()
    pc = generator.get()
    cwipc_write_debugdump('output.cwipcdump', pc)
    cwipc_write('output.ply', pc)
    
if __name__ == '__main__':
    main()
    
    
