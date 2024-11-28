from __future__ import annotations
import ctypes
import ctypes.util
import warnings
import os
import sys
import open3d
import numpy
import numpy.typing
from typing import Optional, List, Type, Any, Union, Dict
from .abstract import cwipc_abstract, cwipc_source_abstract, cwipc_tiledsource_abstract, cwipc_tileinfo_pythonic

__all__ = [
    'CWIPC_API_VERSION',
    'CWIPC_POINT_PACKETHEADER_MAGIC',
    'CWIPC_FLAGS_BINARY',
    'CwipcError',
    '_cwipc_dll_search_path_collection',
    
    'cwipc_wrapper',
    'cwipc_source_wrapper',
    'cwipc_tiledsource_wrapper',
    
    'cwipc_point',
    'cwipc_point_array',
    'cwipc_point_numpy_dtype',

    'cwipc_tileinfo_pythonic',
    
    'cwipc_point_packetheader',
    
    'cwipc_get_version',
    'cwipc_read',
    'cwipc_read_debugdump',
    'cwipc_write',
    'cwipc_write_debugdump',
    'cwipc_from_points',
    'cwipc_from_packet',
    'cwipc_from_certh',
    'cwipc_from_numpy_array',
    'cwipc_from_numpy_matrix',
    'cwipc_from_o3d_pointcloud',
    
    'cwipc_synthetic',
    'cwipc_capturer',
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

CWIPC_API_VERSION = 0x20240128

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
_CWIPC_DEBUG_DLL_SEARCH_PATH=os.getenv("_CWIPC_DEBUG_DLL_SEARCH_PATH")

_WINDOWS_NEEDED_DLLS=[
    "pcl_common",
    "vtkCommonCore-8.2",
    "OpenNI2",
]

class _cwipc_dll_search_path_collection:

    open_dll_dirs : List[Any]

    """Hack to ensure the correct DLL search path is used when loading a DLL on Windows"""
    def __init__(self, dlls : Optional[List[str]]=None):
        # On Darwin we need to add directories to the find_library() search path
        if os.name == "posix" and sys.platform == "darwin":
            self._darwin_add_dylib_search_path()
        if os.name == "posix" and sys.platform == "linux":
            self._linux_add_so_search_path()

        additional_path_entries = []
        if 'CWIPC_LIBRARY_DIR' in os.environ:
            libpath = os.environ['CWIPC_LIBRARY_DIR']
            additional_path_entries.append(libpath)
            if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: add {libpath} from CWIPC_LIBRARY_DIR", file=sys.stderr)

        # On Windows we may need to add directories with add_dll_directory() so we can find dependent DLLs
        self.open_dll_dirs = []
        if not hasattr(os, 'add_dll_directory'):
            # Apparently we don't need to do this...
            if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: not needed", file=sys.stderr)
            return
        if dlls:
            # We implicitly add DLLs needed by cwipc_util
            path_entries = self._get_dll_directories(dlls + _WINDOWS_NEEDED_DLLS)
        else:
            path_string = os.environ.get('PATH')
            assert path_string
            path_entries = additional_path_entries + path_string.split(os.pathsep)
        if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: {len(path_entries)} on PATH", file=sys.stderr)
        for p in path_entries:
            if not os.path.exists(p):
                if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: not found: {p}", file=sys.stderr)
                continue
            try:
                self.open_dll_dirs.append(os.add_dll_directory(p)) # type: ignore
                if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: add_dll_directory {p}", file=sys.stderr)
            except FileNotFoundError as e:
                warnings.warn(f'cwipc_dll_search_path_collection: {e}')

    def find_library(self, libname : str) -> Optional[str]:
        rv = ctypes.util.find_library(libname)
        if rv and not os.path.isabs(rv):
            # Assume we are on Linux (which returns non-absolute paths from find_library)
            # Attempt to find it on LD_LIBRARY_PATH, which ld.so does not honour changes to during runtime.
            paths = os.environ.get('LD_LIBRARY_PATH', '')
            for p in paths.split(':'):
                if p:
                    attempt = os.path.join(p, rv)
                    if os.path.exists(attempt):
                        rv = attempt
                        break
        return rv
        
    def _darwin_add_dylib_search_path(self) -> None:
        """Add lib directories in our ancestors to DYLD_LIBRARY_PATH so _our_ dylibs are found by find_library"""
        if 'DYLD_LIBRARY_PATH' in os.environ:
            return
        candidates : List[str] = []
        # Add any override directory from the environment (mainly for use from within vscode, for development)
        if 'CWIPC_LIBRARY_DIR' in os.environ:
            libpath = os.environ['CWIPC_LIBRARY_DIR']
            candidates.append(libpath)
            if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: add_dylib_search_directory {libpath} from CWIPC_LIBRARY_DIR", file=sys.stderr)
        # Add all "lib" directories found in our ancestors
        basepath = os.path.dirname(__file__)
        while basepath:
            libpath = os.path.join(basepath, 'lib')
            if os.path.isdir(libpath):
                if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: add_dylib_search_directory {libpath}", file=sys.stderr)
                candidates.append(libpath)
            nextbasepath = os.path.dirname(basepath)
            if nextbasepath == basepath:
                break
            basepath = nextbasepath
        os.environ['DYLD_LIBRARY_PATH'] = ':'.join(candidates)
        
    def _linux_add_so_search_path(self) -> None:
        """Add lib directories in our ancestors to LD_LIBRARY_PATH so _our_ shared objects are found by find_library"""
        if 'LD_LIBRARY_PATH' in os.environ:
            return
        candidates : List[str] = []
        # Add any override directory from the environment (mainly for use from within vscode, for development)
        if 'CWIPC_LIBRARY_DIR' in os.environ:
            libpath = os.environ['CWIPC_LIBRARY_DIR']
            candidates.append(libpath)
            if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: add_so_search_directory {libpath} from CWIPC_LIBRARY_DIR", file=sys.stderr)
        # Add all "lib" directories found in our ancestors
        basepath = os.path.dirname(__file__)
        while basepath:
            libpath = os.path.join(basepath, 'lib')
            if os.path.isdir(libpath):
                if _CWIPC_DEBUG_DLL_SEARCH_PATH: print(f"_cwipc_dll_search_path_collection: add_so_search_directory {libpath}", file=sys.stderr)
                candidates.append(libpath)
            nextbasepath = os.path.dirname(basepath)
            if nextbasepath == basepath:
                break
            basepath = nextbasepath
        os.environ['LD_LIBRARY_PATH'] = ':'.join(candidates)
        
    def _get_dll_directories(self, dlls : List[str]) -> List[str]:
        """Return a list of directory names that contain all DLLs pased ar argument"""
        done : List[str] = []
        rv : List[str] = []
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
        
    def __exit__(self, type: Optional[Type[BaseException]], value: Optional[BaseException], traceback: Optional[Any]) -> Optional[bool]:
        for d in self.open_dll_dirs:
            d.close()
            
class CwipcError(RuntimeError):
    pass
    
_cwipc_util_dll_reference = None

class cwipc_p(ctypes.c_void_p):
    """ctypes-compatible native pointer to a cwipc object"""
    pass
    
class cwipc_source_p(ctypes.c_void_p):
    """ctypes-compatible native pointer to a cwipc_source object"""
    pass

class cwipc_tiledsource_p(cwipc_source_p):
    """ctypes-compatible native pointer to a cwipc_tiledsource object"""
    pass

class cwipc_sink_p(ctypes.c_void_p):
    """ctypes-compatible native pointer to a cwipc_sink object"""
    pass

class cwipc_auxiliary_data_p(ctypes.c_void_p):
    """ctypes-compatible native pointer to a cwipc_auxiliary_data object"""
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
    
    def __eq__(self, other : Any) -> bool:
        if not isinstance(other, cwipc_point):
            return False
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return False
        return True

    def __ne__(self, other : Any) -> bool:
        if not isinstance(other, cwipc_point):
            return True
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return True
        return False

# Pythonic representation of a cwipc_point
cwipc_point_tuple = tuple[float, float, float, int, int, int, int]
# Numpy dtype definition of a cwipc_point
cwipc_point_numpy_dtype = [('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('tile', 'u1')]

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
    
    def __eq__(self, other : Any) -> bool:
        if not isinstance(other, cwipc_vector):
            return False
        for fld in self._fields_:
            if getattr(self, fld[0]) != getattr(other, fld[0]):
                return False
        return True

    def __ne__(self, other : Any) -> bool:
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
def cwipc_util_dll_load(libname : Optional[str]=None) -> ctypes.CDLL:
    """Load the cwipc_util DLL and assign the signatures (if not already loaded).
    
    If you want to load a non-default native library (for example to allow debugging low level code)
    call this method early, before any other method from this package.
    """
    global _cwipc_util_dll_reference
    if _cwipc_util_dll_reference: return _cwipc_util_dll_reference
    
    with _cwipc_dll_search_path_collection(None) as loader:
        if libname == None:
            libname = 'cwipc_util'
        if not os.path.isabs(libname):
            libname = loader.find_library(libname)
            if not libname:
                raise RuntimeError('Dynamic library cwipc_util not found')
        assert libname
        _cwipc_util_dll_reference = ctypes.CDLL(libname)
    
    _cwipc_util_dll_reference.cwipc_get_version.argtypes = []
    _cwipc_util_dll_reference.cwipc_get_version.restype = ctypes.c_char_p
    
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
    
    _cwipc_util_dll_reference.cwipc_tiledsource_reload_config.argtypes = [cwipc_tiledsource_p, ctypes.c_char_p]
    _cwipc_util_dll_reference.cwipc_tiledsource_reload_config.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_tiledsource_get_config.argtypes = [cwipc_tiledsource_p, ctypes.POINTER(ctypes.c_byte), ctypes.c_size_t]
    _cwipc_util_dll_reference.cwipc_tiledsource_get_config.restype = ctypes.c_size_t
    
    _cwipc_util_dll_reference.cwipc_tiledsource_seek.argtypes = [cwipc_tiledsource_p, ctypes.c_uint64]
    _cwipc_util_dll_reference.cwipc_tiledsource_seek.restype = ctypes.c_bool
    
    _cwipc_util_dll_reference.cwipc_tiledsource_maxtile.argtypes = [cwipc_tiledsource_p]
    _cwipc_util_dll_reference.cwipc_tiledsource_maxtile.restype = ctypes.c_int
    
    _cwipc_util_dll_reference.cwipc_tiledsource_get_tileinfo.argtypes = [cwipc_tiledsource_p, ctypes.c_int, ctypes.POINTER(cwipc_tileinfo)]
    _cwipc_util_dll_reference.cwipc_tiledsource_get_tileinfo.restype = ctypes.c_int

    _cwipc_util_dll_reference.cwipc_tiledsource_auxiliary_operation.argtypes = [cwipc_source_p, ctypes.c_char_p, ctypes.POINTER(ctypes.c_byte), ctypes.c_size_t, ctypes.POINTER(ctypes.c_byte), ctypes.c_size_t]
    _cwipc_util_dll_reference.cwipc_tiledsource_auxiliary_operation.restype = ctypes.c_bool
    
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

    _cwipc_util_dll_reference.cwipc_capturer.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_char_p), ctypes.c_ulong]
    _cwipc_util_dll_reference.cwipc_capturer.restype = cwipc_tiledsource_p

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

cwipc_point_array_value_type = Union[None, bytearray, bytes, ctypes.Array[cwipc_point], List[tuple[float, float, float, int, int, int, int]]]

def cwipc_point_array(*, count : Optional[int]=None, values : Any=()) -> ctypes.Array[cwipc_point]:
    """Create an array of cwipc_point elements. `count` can be specified, or `values` can be a tuple or list of tuples (x, y, z, r, g, b, tile), or both"""
    if count == None:
        count = len(values)
    allocator = cwipc_point * count
    if isinstance(values, bytearray):
        return allocator.from_buffer(values)
    elif isinstance(values, bytes):
        return allocator.from_buffer_copy(values)
    if not isinstance(values, tuple):
        values = tuple(values)
    return allocator(*values)
    
cwipc_point_numpy_array_value_type = numpy.typing.NDArray[Any]
cwipc_point_numpy_matrix_value_type = numpy.typing.NDArray[numpy.floating]

class cwipc_wrapper(cwipc_abstract):
    """Pointcloud as an opaque object."""
    
    _cwipc : Optional[cwipc_p]
    _bytes : Optional[bytearray]
    _points : Optional[ctypes.Array[cwipc_point]]

    def __init__(self, _cwipc : Optional[cwipc_p]=None):
        if _cwipc != None:
            assert isinstance(_cwipc, cwipc_p)
        self._cwipc = _cwipc
        self._points = None
        self._bytes = None
        
    def as_cwipc_p(self) -> cwipc_p:
        """Return ctypes-compatible pointer for this object"""
        assert self._cwipc
        return self._cwipc
            
    def free(self) -> None:
        """Delete the opaque pointcloud object (by asking the original creator to do so)"""
        if self._cwipc:
            cwipc_util_dll_load().cwipc_free(self.as_cwipc_p())
        self._cwipc = None
        
    def timestamp(self) -> int:
        """Returns timestamp (microseconds) when this pointcloud was captured (relative to some unspecified origin)"""
        rv = cwipc_util_dll_load().cwipc_timestamp(self.as_cwipc_p())
        return rv
        
    def cellsize(self) -> float:
        """Returns the size of the cells this pointcloud represents (0 if unknown)"""
        rv = cwipc_util_dll_load().cwipc_cellsize(self.as_cwipc_p())
        return rv
        
    def _set_cellsize(self, cellsize : float) -> None:
        """Internal use only: set the size of the cells this pointcloud represents"""
        cwipc_util_dll_load().cwipc__set_cellsize(self.as_cwipc_p(), cellsize)
        
    def _set_timestamp(self, timestamp : int) -> None:
        """Internal use only: set the size of the cells this pointcloud represents"""
        cwipc_util_dll_load().cwipc__set_timestamp(self.as_cwipc_p(), timestamp)
        
    def count(self) -> int:
        """Get the number of points in the pointcloud"""
        rv = cwipc_util_dll_load().cwipc_count(self.as_cwipc_p())
        return rv
        
    def get_uncompressed_size(self) -> int:
        """Get the size in bytes of the uncompressed pointcloud data"""
        rv = cwipc_util_dll_load().cwipc_get_uncompressed_size(self.as_cwipc_p())
        return rv
        
    def get_points(self) -> ctypes.Array[cwipc_point]:
        """Get the pointcloud data as a cwipc_point_array"""
        if self._points == None:
            self._initialize_points_and_bytes()
        assert self._points != None
        return self._points
        
    def get_numpy_array(self) -> cwipc_point_numpy_array_value_type:
        """Return the pointcloud data as a numpy array of records"""
        points = self.get_points()
        np_points = numpy.ctypeslib.as_array(points)
        return np_points
    
    def get_numpy_matrix(self, onlyGeometry=False) -> cwipc_point_numpy_matrix_value_type:
        """Return the pointcloud as a numpy matrix of floats, shape Nx7, columns x, y, z, r, g, b, tile.
        If onlyGeometry is True only the first three columns will be returned.
        """
        points = self.get_points()
        np_points = numpy.ctypeslib.as_array(points)
        nColumn = 3 if onlyGeometry else 7
        np_shape = (np_points.shape[0], nColumn)
        np_matrix = numpy.zeros(np_shape, numpy.float32)
        np_matrix[..., 0] = np_points['x']
        np_matrix[..., 1] = np_points['y']
        np_matrix[..., 2] = np_points['z']
        
        if not onlyGeometry:
            np_rgbt_shape = (np_points.shape[0], 4)
            np_rgbt = numpy.zeros(np_rgbt_shape, numpy.uint8)
            np_rgbt[:,0] = np_points['r']
            np_rgbt[:,1] = np_points['g']
            np_rgbt[:,2] = np_points['b']
            np_rgbt[:,3] = np_points['tile']
            
            np_rgbt_float = np_rgbt.astype(numpy.float32)
            np_matrix[..., 3:] = np_rgbt_float
        return np_matrix

    def get_o3d_pointcloud(self) -> open3d.geometry.PointCloud:
        """Returns an Open3D PointCloud representing this cwipc pointcloud"""
        np_matrix = self.get_numpy_matrix()
        points = np_matrix[:,0:3]
        colors = np_matrix[:,3:6] / 255.0
        o3d_pc = open3d.geometry.PointCloud()
        o3d_points = open3d.utility.Vector3dVector(points)
        o3d_pc.points = o3d_points
        o3d_colors = open3d.utility.Vector3dVector(colors)
        o3d_pc.colors = o3d_colors
        return o3d_pc
        
    def get_bytes(self) -> bytearray:
        """Get the pointcloud data as Python bytes"""
        if self._bytes == None:
            self._initialize_points_and_bytes()
        assert self._bytes
        return self._bytes
        
    def access_auxiliary_data(self):
        rv_p = cwipc_util_dll_load().cwipc_access_auxiliary_data(self.as_cwipc_p())
        if rv_p:
            return cwipc_auxiliary_data(rv_p)
        return None
        
    def _initialize_points_and_bytes(self) -> None:
        assert self._cwipc
        nBytes : int = cwipc_util_dll_load().cwipc_get_uncompressed_size(self.as_cwipc_p())
        buffer = bytearray(nBytes)
        bufferCtypesType = ctypes.c_byte * nBytes
        bufferArg = bufferCtypesType.from_buffer(buffer)
        nPoints = cwipc_util_dll_load().cwipc_copy_uncompressed(self.as_cwipc_p(), bufferArg, nBytes)
        points = cwipc_point_array(count=nPoints, values=buffer)
        self._bytes = buffer
        self._points = points

    def get_packet(self) -> bytearray:
        assert self._cwipc
        nBytes : int = cwipc_util_dll_load().cwipc_copy_packet(self.as_cwipc_p(), None, 0)
        buffer = bytearray(nBytes)
        bufferCtypesType = ctypes.c_byte * nBytes
        bufferArg = bufferCtypesType.from_buffer(buffer)
        rvNBytes = cwipc_util_dll_load().cwipc_copy_packet(self.as_cwipc_p(), bufferArg, nBytes)
        assert rvNBytes == nBytes
        return buffer

class cwipc_source_wrapper(cwipc_source_abstract):
    """Pointcloud source as an opaque object"""
    _cwipc_source : Optional[cwipc_source_p]

    def __init__(self, _cwipc_source : Optional[cwipc_source_p]=None):
        if _cwipc_source != None:
            assert isinstance(_cwipc_source, cwipc_source_p)
        self._cwipc_source = _cwipc_source

    def as_cwipc_source_p(self) -> cwipc_source_p:
        """Return ctypes-compatible pointer for this object"""
        assert self._cwipc_source
        return self._cwipc_source
            
    def free(self) -> None:
        """Delete the opaque pointcloud source object (by asking the original creator to do so)"""
        if self._cwipc_source:
            cwipc_util_dll_load().cwipc_source_free(self.as_cwipc_source_p())
        self._cwipc_source = None
        
    def eof(self) -> bool:
        """Return True if no more pointclouds will be forthcoming"""
        return cwipc_util_dll_load().cwipc_source_eof(self.as_cwipc_source_p())
        
    def available(self, wait : bool) -> bool:
        """Return True if a pointcloud is currently available. The wait parameter signals the source may wait a while."""
        return cwipc_util_dll_load().cwipc_source_available(self.as_cwipc_source_p(), wait)
        
    def get(self) -> Optional[cwipc_wrapper]:
        """Get a cwipc (opaque pointcloud) from this source. Returns None if no more pointcloudes are forthcoming"""
        rv = cwipc_util_dll_load().cwipc_source_get(self.as_cwipc_source_p())
        if rv:
            return cwipc_wrapper(rv)
        return None

    def request_auxiliary_data(self, name : str) -> None:
        """Ask this grabber to also provide auxiliary data `name` with each pointcloud"""
        cname = name.encode('utf8')
        return cwipc_util_dll_load().cwipc_source_request_auxiliary_data(self.as_cwipc_source_p(), cname)

    def auxiliary_data_requested(self, name : str) -> bool:
        """Return True if this grabber provides auxiliary data `name` with each pointcloud"""
        cname = name.encode('utf8')
        return cwipc_util_dll_load().cwipc_source_auxiliary_data_requested(self.as_cwipc_source_p(), cname)
        
        
class cwipc_tiledsource_wrapper(cwipc_source_wrapper, cwipc_tiledsource_abstract):
    """Tiled pointcloud sources as opaque object"""
    _cwipc_source : Optional[cwipc_tiledsource_p]

    def __init__(self, _cwipc_tiledsource : Optional[cwipc_tiledsource_p]=None):
        if _cwipc_tiledsource != None:
            assert isinstance(_cwipc_tiledsource, cwipc_tiledsource_p)
        self._cwipc_source = _cwipc_tiledsource
        
    def reload_config(self, config : Union[str, bytes, None]) -> None:
        """Load a config from file or JSON string"""
        if type(config) == str:
            config = config.encode('utf8')
        return cwipc_util_dll_load().cwipc_tiledsource_reload_config(self.as_cwipc_source_p(), config)
        
    def get_config(self) -> bytes:
        """Return current capturer cameraconfig as JSON"""
        nBytes = cwipc_util_dll_load().cwipc_tiledsource_get_config(self.as_cwipc_source_p(), None, 0)
        if nBytes <= 0:
            raise CwipcError("this cwipc_tiledsource has no camera configuration")
        buffer = bytearray(nBytes)
        bufferCtypesType = ctypes.c_byte * nBytes
        bufferArg = bufferCtypesType.from_buffer(buffer)
        rvNBytes = cwipc_util_dll_load().cwipc_tiledsource_get_config(self.as_cwipc_source_p(), bufferArg, nBytes)
        assert rvNBytes == nBytes
        return buffer
        
    def seek(self, timestamp : int) -> bool:
        """Return true if seek was successfull"""
        return cwipc_util_dll_load().cwipc_tiledsource_seek(self.as_cwipc_source_p(),timestamp)
    
    def maxtile(self) -> int:
        """Return maximum number of tiles creatable from cwipc objects generated by this source"""
        return cwipc_util_dll_load().cwipc_tiledsource_maxtile(self.as_cwipc_source_p())

    def get_tileinfo_raw(self, tilenum : int) -> Optional[cwipc_tileinfo]:
        """Return cwipc_tileinfo for tile tilenum, or None"""
        info = cwipc_tileinfo()
        rv = cwipc_util_dll_load().cwipc_tiledsource_get_tileinfo(self.as_cwipc_source_p(), tilenum, ctypes.byref(info))
        if not rv:
            return None
        return info
        
    def get_tileinfo_dict(self, tilenum : int) -> cwipc_tileinfo_pythonic:
        """Return tile information for tile tilenum as Python dictionary"""
        info = self.get_tileinfo_raw(tilenum)
        if info == None:
            raise CwipcError(f"get_tileinfo_raw({tilenum}) returned None")
        normal = dict(x=info.normal.x, y=info.normal.y, z=info.normal.z)
        return dict(normal=normal, cameraName=info.cameraName, ncamera=info.ncamera, cameraMask=info.cameraMask)
        
    def auxiliary_operation(self, op : str, inbuf : bytes, outbuf : bytearray) -> bool:
        """Perform operation in the capturer."""
        c_op = op.encode('utf8')
        c_inbuf_size = len(inbuf)
        c_inbuf_type = ctypes.c_byte * c_inbuf_size
        c_inbuf = c_inbuf_type.from_buffer_copy(inbuf)
        c_outbuf_size = len(outbuf)
        c_outbuf_type = ctypes.c_byte * c_outbuf_size
        c_outbuf = c_outbuf_type.from_buffer(outbuf)
        return cwipc_util_dll_load().cwipc_tiledsource_auxiliary_operation(self.as_cwipc_source_p(), c_op, c_inbuf, c_inbuf_size, c_outbuf, c_outbuf_size)
   
class cwipc_sink_wrapper:
    """Pointcloud sink as an opaque object"""
    _cwipc_sink : Optional[cwipc_sink_p]

    def __init__(self, _cwipc_sink : Optional[cwipc_sink_p]=None):
        if _cwipc_sink != None:
            assert isinstance(_cwipc_sink, cwipc_sink_p)
        self._cwipc_sink = _cwipc_sink

    def as_cwipc_sink_p(self) -> cwipc_sink_p:
        """Return ctypes-compatible pointer for this object"""
        assert self._cwipc_sink
        return self._cwipc_sink
            
    def free(self) -> None:
        """Delete the opaque pointcloud sink object (by asking the original creator to do so)"""
        if self._cwipc_sink:
            cwipc_util_dll_load().cwipc_sink_free(self.as_cwipc_sink_p())
        self._cwipc_source = None
        
    def feed(self, pc : Optional[cwipc_wrapper], clear : bool) -> bool:
        if pc == None:
            cpc = None
        else:
            cpc = pc.as_cwipc_p() # type: ignore
        return cwipc_util_dll_load().cwipc_sink_feed(self.as_cwipc_sink_p(), cpc, clear)
        
    def caption(self, caption : str) -> None:
        return cwipc_util_dll_load().cwipc_sink_caption(self.as_cwipc_sink_p(), caption.encode('utf8'))
        
    def interact(self, prompt : Optional[str], responses : Optional[str], millis : int) -> str:
        cprompt : Optional[bytes] = None
        cresponses : Optional[bytes] = None
        if prompt != None: cprompt = prompt.encode('utf8')
        if responses != None: cresponses = responses.encode('utf8')
        rv = cwipc_util_dll_load().cwipc_sink_interact(self.as_cwipc_sink_p(), cprompt, cresponses, millis)
        return rv.decode('utf8')
        
class cwipc_auxiliary_data:
    """Additional data attached to a cwipc object"""

    _cwipc_auxiliary_data : Optional[cwipc_auxiliary_data_p]

    def __init__(self, _cwipc_auxiliary_data : Optional[cwipc_auxiliary_data_p]=None):
        if _cwipc_auxiliary_data != None:
            assert isinstance(_cwipc_auxiliary_data, cwipc_auxiliary_data_p)
        self._cwipc_auxiliary_data = _cwipc_auxiliary_data

    def as_cwipc_auxiliary_data_p(self) -> cwipc_auxiliary_data_p:
        """Return ctypes-compatible pointer for this object"""
        assert self._cwipc_auxiliary_data
        return self._cwipc_auxiliary_data
        
    def count(self) -> int:
        """Return number of items in auxiliary_data collection"""
        return cwipc_util_dll_load().cwipc_auxiliary_data_count(self.as_cwipc_auxiliary_data_p())
        
    def name(self, idx : int) -> str:
        """Return name of item idx"""
        rv = cwipc_util_dll_load().cwipc_auxiliary_data_name(self.as_cwipc_auxiliary_data_p(), idx)
        return rv.decode('utf8')
        
    def description(self, idx : int) -> str:
        """Return description string of item idx"""
        rv = cwipc_util_dll_load().cwipc_auxiliary_data_description(self.as_cwipc_auxiliary_data_p(), idx)
        return rv.decode('utf8')
        
    def pointer(self, idx : int) -> ctypes.c_void_p:
        """Access native point to item idx"""
        return cwipc_util_dll_load().cwipc_auxiliary_data_pointer(self.as_cwipc_auxiliary_data_p(), idx)
        
    def size(self, idx : int) -> int:
        """Return size if bytes of item idx"""
        return cwipc_util_dll_load().cwipc_auxiliary_data_size(self.as_cwipc_auxiliary_data_p(), idx)
        
    def data(self, idx : int) -> bytes:
        """Return data of item idx as bytes"""
        size = self.size(idx)
        pointer = self.pointer(idx)
        c_type = ctypes.c_ubyte*size
        c_array = c_type.from_address(pointer) # type: ignore
        rv = bytearray(c_array)
        return rv
    
    # Auxiliary methods to get image data from auxiliary data
    def _parse_aux_description(self, description : str) -> Dict[str, Any]:
        rv = {}
        fields = description.split(',')
        for f in fields:
            k, v = f.split('=')
            try:
                v = int(v)
            except ValueError:
                pass
            rv[k] = v
        return rv
    
    def get_image_description(self, idx : int) -> Dict[str, Any]:
        """Parse description of item idx as an image description"""
        desc_str = self.description(idx)
        desc = self._parse_aux_description(desc_str)
        # Add some more useful values
        if "bpp" in desc:
            bpp = desc["bpp"]
            if bpp == 2:
                desc["image_format"] = "Z16"
            elif bpp == 3:
                desc["image_format"] = "RGB8"
            elif bpp == 4:
                desc["image_format"] = "RGBA"
        if "format" in desc:
            image_format = desc['format']
            if image_format == 2:
                desc["bpp"] = 3
                desc["image_format"] = "RGB8"
            elif image_format == 3:
                desc["bpp"] = 4 # RGBA
                desc["image_format"] = "BGRA"
            elif image_format == 4:
                desc["bpp"] = 2 # 16-bit grey
                desc["image_format"] = "Z16"
            else:
                # This caters for newer realsense code, which puts a string format specifier into the format parameter.
                # Note that this will override any format surmised by looking at bpp
                desc["image_format"] = image_format
        return desc

    def get_image(self, idx : int) -> numpy.typing.NDArray[Any]:
        """Return item idx, which should be an image, as a numpy NDArray.
        If the item is an RGB image it will be width*height*3, uint8 B, G, R.
        If the item is a Depth image it will be width*height, uint16.
        """
        descr = self.get_image_description(idx)
        image_format = descr["image_format"]
        image_data = self.data(idx)
        if image_format == "Z16":
            np_image_data_raw = numpy.frombuffer(image_data, numpy.uint16)
            shape = (descr["height"], descr["width"])
            np_image_data = numpy.reshape(np_image_data_raw, shape)
        elif image_format == "RGB8":
            np_image_data_bytes = numpy.frombuffer(image_data, numpy.uint8)
            shape = (descr["height"], descr["width"], descr["bpp"])
            np_image_data = numpy.reshape(np_image_data_bytes, shape)
            np_image_data = np_image_data[:,:,[2,1,0]]
        elif image_format == "BGRA":
            np_image_data_bytes = numpy.frombuffer(image_data, numpy.uint8)
            shape = (descr["height"], descr["width"], descr["bpp"])
            np_image_data = numpy.reshape(np_image_data_bytes, shape)
            np_image_data = np_image_data[:,:,[0,1,2]]
        else:
            raise CwipcError(f"Unknown auxiliary data image format: {repr(image_format)}")
        return np_image_data
    
    def get_all_images(self, pattern : str = "") -> Dict[str, numpy.typing.NDArray[Any]]:
        """Return all images as a dictionary mapping name to the numpy image array.
        The optional pattern argument can be used to limit which images to get (name must contain pattern), and
        that string will be removed from the image name. (In other words: passing ".12345" will get "rgb" and "depth"
        for that serial number, and passing "rgb." will get the serial number).

        If the item is an RGB image it will be width*height*3, uint8 B, G, R.
        If the item is a Depth image it will be width*height, uint16.
        """
        rv = {}
        for idx in range(self.count()):
            name = self.name(idx)
            if not name.startswith("rgb.") and not name.startswith("depth."):
                continue
            if pattern:
                if not pattern in name:
                    continue
                name = name.replace(pattern, '')

            image = self.get_image(idx)
            rv[name] = image
        return rv
   
def cwipc_get_version() -> str:
    """Return version information"""
    c_version = cwipc_util_dll_load().cwipc_get_version()
    version = c_version.decode('utf8')
    try:
        import pkg_resources
        py_version = pkg_resources.require("cwipc_util")[0].version
        if py_version and py_version != version:
            version = f'{version} (python wrapper: {py_version})'
    except ImportError:
        pass
    return version
                 
def cwipc_read(filename : str, timestamp : int) -> cwipc_wrapper:
    """Read pointcloud from a .ply file, return as cwipc object. Timestamp must be passsed in too."""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_read(filename.encode('utf8'), timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString and errorString.value:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc_wrapper(rv)
    raise CwipcError("cwipc_read: no pointcloud read, but no specific error returned from C library")
    
def cwipc_write(filename : str, pointcloud : cwipc_wrapper, flags : int=0) -> int:
    """Write a cwipc object to a .ply file."""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_write_ext(filename.encode('utf8'), pointcloud.as_cwipc_p(), flags, ctypes.byref(errorString))
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    return rv

def cwipc_from_points(points : cwipc_point_array_value_type, timestamp : int) -> cwipc_wrapper:
    """Create a cwipc from either `cwipc_point_array` or a list or tuple of xyzrgb values"""
    if not isinstance(points, ctypes.Array):
        points = cwipc_point_array(values=points)
    addr = ctypes.addressof(points)
    nPoint = len(points)
    nBytes = ctypes.sizeof(points)
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_from_points(addr, nBytes, nPoint, timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_wrapper(rv)
    raise CwipcError("cwipc_from_points: cannot create cwipc from given argument")

def cwipc_from_numpy_array(np_points : cwipc_point_numpy_array_value_type, timestamp : int) -> cwipc_wrapper:
    """Create a cwipc from either `cwipc_point_array` or a list or tuple of xyzrgb values"""
    nPoint = np_points.shape[0]
    addr = np_points.ctypes.data_as(ctypes.POINTER(cwipc_point))
    nBytes = nPoint * np_points.strides[0]
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_from_points(addr, nBytes, nPoint, timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_wrapper(rv)
    raise CwipcError("cwipc_from_numpy_array: cannot create cwipc from given argument")


def cwipc_from_numpy_matrix(np_points_matrix : cwipc_point_numpy_matrix_value_type, timestamp : int) -> cwipc_wrapper:
    """Create a cwipc from either `cwipc_point_array` or a list or tuple of xyzrgb values"""
    count = np_points_matrix.shape[0]
    assert np_points_matrix.shape == (count, 7)
    assert np_points_matrix.dtype in (numpy.float32, numpy.float64)
    np_points = numpy.zeros(count, cwipc_point_numpy_dtype)
    np_points['x'] = np_points_matrix[:,0]
    np_points['y'] = np_points_matrix[:,1]
    np_points['z'] = np_points_matrix[:,2]
    np_points['r'] = np_points_matrix[:,3].astype(numpy.uint8)
    np_points['g'] = np_points_matrix[:,4].astype(numpy.uint8)
    np_points['b'] = np_points_matrix[:,5].astype(numpy.uint8)
    np_points['tile'] = np_points_matrix[:,6].astype(numpy.uint8)
    return cwipc_from_numpy_array(np_points, timestamp)

def cwipc_from_o3d_pointcloud(o3d_pc : open3d.geometry.PointCloud, timestamp : int) -> cwipc_wrapper:
    points = numpy.asarray(o3d_pc.points)
    colors = numpy.asarray(o3d_pc.colors)
    nPoints = points.shape[0]
    np_matrix = numpy.zeros((nPoints, 7))
    np_matrix[..., 0:3] = points
    np_matrix[..., 3:6] = colors * 256
    # We have no way to construct tilenum
    return cwipc_from_numpy_matrix(np_matrix, timestamp)

def cwipc_from_packet(packet : bytes) -> cwipc_wrapper:
    nBytes = len(packet)
    byte_array_type = ctypes.c_char * nBytes
    try:
        c_packet = byte_array_type.from_buffer(packet)
    except TypeError:
        c_packet = byte_array_type.from_buffer_copy(packet)
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_from_packet(c_packet, nBytes, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_wrapper(rv)
    raise CwipcError("cwipc_from_packet: no pointcloud read, but no specific error returned from C library")

def cwipc_from_certh(certhPC : ctypes.c_void_p, timestamp : int, origin : Optional[tuple[float, float, float]]=None, bbox : Optional[tuple[float, float, float, float, float, float]]=None) -> cwipc_wrapper:
    """Create a cwipc from a CERTH PointCloud structure (address passed as ctypes.c_void_p)"""
    corigin = None
    if origin:
        corigin = (ctypes.c_float*3)(*origin)
        corigin = ctypes.cast(corigin, ctypes.c_void_p)
    cbbox = None
    if bbox:
        cbbox = (ctypes.c_float*6)(*bbox)
        cbbox = ctypes.cast(cbbox, ctypes.c_void_p)
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_from_certh(certhPC, corigin, cbbox, timestamp, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_wrapper(rv)
    raise CwipcError("cwipc_from_certh: no pointcloud returned, but no specific error returned from C library")
    
def cwipc_read_debugdump(filename : str) -> cwipc_wrapper:
    """Return a cwipc object read from a .cwipcdump file."""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_read_debugdump(filename.encode('utf8'), ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_wrapper(rv)
    raise CwipcError("cwipc_read_debugdump: no pointcloud read, but no specific error returned from C library")

    
def cwipc_write_debugdump(filename : str, pointcloud : cwipc_wrapper) -> int:
    """Write a cwipc object to a .cwipcdump file."""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_write_debugdump(filename.encode('utf8'), pointcloud.as_cwipc_p(), ctypes.byref(errorString))
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    return rv
    
def cwipc_synthetic(fps : int=0, npoints : int=0) -> cwipc_tiledsource_wrapper:
    """Returns a cwipc_source object that returns synthetically generated cwipc objects on every get() call."""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_synthetic(fps, npoints, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource_wrapper(rv)
    raise CwipcError("cwipc_synthetic: cannot create synthetic source, but no specific error returned from C library")
    
def cwipc_capturer(conffile : Optional[str]=None) -> cwipc_tiledsource_wrapper:
    """Returns a cwipc_source object that grabs from a camera and returns cwipc object on every get() call."""
    errorString = ctypes.c_char_p()
    cconffile = None
    if conffile:
        cconffile = conffile.encode('utf8')
    rv = cwipc_util_dll_load().cwipc_capturer(cconffile, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value and not rv:
        raise CwipcError(errorString.value.decode('utf8'))
    if errorString and errorString.value:
        warnings.warn(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource_wrapper(rv)
    raise CwipcError("cwipc_capturer: cannot create capturer, but no specific error returned from C library")

    
def cwipc_window(title : str) -> cwipc_sink_wrapper:
    """Returns a cwipc_sink object that displays pointclouds in a window"""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_window(title.encode('utf8'), ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_sink_wrapper(rv)
    raise CwipcError("cwipc_window: cannot create window, but no specific error returned from C library")
    
def cwipc_downsample(pc : cwipc_wrapper, voxelsize : float) -> cwipc_wrapper:
    """Return a pointcloud voxelized to cubes with the given voxelsize"""
    rv = cwipc_util_dll_load().cwipc_downsample(pc.as_cwipc_p(), voxelsize)
    return cwipc_wrapper(rv)
    
def cwipc_remove_outliers(pc : cwipc_wrapper, kNeighbors : int, stdDesvMultThresh : float, perTile : bool) -> cwipc_wrapper:
    """Return a pointcloud with outlier points removed."""
    rv = cwipc_util_dll_load().cwipc_remove_outliers(pc.as_cwipc_p(), kNeighbors, stdDesvMultThresh, perTile)
    return cwipc_wrapper(rv)
    
def cwipc_tilefilter(pc : cwipc_wrapper, tile : int) -> cwipc_wrapper:
    """Return pointcloud with only points that have the given tilenumber"""
    rv = cwipc_util_dll_load().cwipc_tilefilter(pc.as_cwipc_p(), tile)
    return cwipc_wrapper(rv)
  
def cwipc_tilemap(pc : cwipc_wrapper, mapping : Union[List[int], dict[int,int], bytes]) -> cwipc_wrapper:
    """Retur pointcloud with every point tilenumber changed. Mapping can be a list or bytes with 256 entries or a dictionary."""
    if type(mapping) != bytes and type(mapping) != bytearray and type(mapping) != list:
        m = [0]*256
        for k in mapping:
            m[k] = mapping[k]
        mapping = m
    rv = cwipc_util_dll_load().cwipc_tilemap(pc.as_cwipc_p(), bytes(mapping))
    return cwipc_wrapper(rv)
  
def cwipc_colormap(pc : cwipc_wrapper, clearBits : int, setBits : int) -> cwipc_wrapper:
    """Return pointcloud with every point color changed. clearBits and setBits are bitmasks."""
    rv = cwipc_util_dll_load().cwipc_colormap(pc.as_cwipc_p(), clearBits, setBits)
    return cwipc_wrapper(rv)
  
def cwipc_crop(pc : cwipc_wrapper, bbox : Union[tuple[float, float, float, float, float, float], List[float]]) -> cwipc_wrapper:
    """Return pointcloud cropped to a bounding box specified as minx, maxx, miny, maxy, minz, maxz"""
    bbox_arg = (ctypes.c_float*6)(*bbox)
    rv = cwipc_util_dll_load().cwipc_crop(pc.as_cwipc_p(), bbox_arg)
    return cwipc_wrapper(rv)
  
def cwipc_join(pc1 : cwipc_wrapper, pc2 : cwipc_wrapper) -> cwipc_wrapper:
    """Return a pointcloud that is the union of the two arguments"""
    rv = cwipc_util_dll_load().cwipc_join(pc1.as_cwipc_p(), pc2.as_cwipc_p())
    return cwipc_wrapper(rv)
  
def cwipc_proxy(host : str, port : int) -> cwipc_tiledsource_wrapper:
    """Returns a cwipc_source object that starts a server and receives pointclouds over a socket connection"""
    errorString = ctypes.c_char_p()
    rv = cwipc_util_dll_load().cwipc_proxy(host.encode('utf8'), port, ctypes.byref(errorString), CWIPC_API_VERSION)
    if errorString and errorString.value:
        raise CwipcError(errorString.value.decode('utf8'))
    if rv:
        return cwipc_tiledsource_wrapper(rv)
    raise CwipcError("cwipc_proxy: cannot create capturer, but no specific error returned from C library")
