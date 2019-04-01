import ctypes
import ctypes.util
from .util import CwipcError, cwipc_source
from .util import cwipc_source_p

__all__ = [
    "cwpic_realsense2"
]
    
_cwipc_realsense2_dll_reference = None

#
# NOTE: the signatures here must match those in cwipc_util/api.h or all hell will break loose
#
def _cwipc_realsense2_dll(libname=None):
    """Load the cwipc_util DLL and assign the signatures (if not already loaded)"""
    global _cwipc_realsense2_dll_reference
    if _cwipc_realsense2_dll_reference: return _cwipc_realsense2_dll_reference
    
    if libname == None:
        libname = ctypes.util.find_library('cwipc_realsense2')
        if not libname:
            raise RuntimeError('Dynamic library cwipc_realsense2 not found')
    assert libname
    _cwipc_realsense2_dll_reference = ctypes.CDLL(libname)
    
    _cwipc_realsense2_dll_reference.cwipc_realsense2.argtypes = []
    _cwipc_realsense2_dll_reference.cwipc_realsense2.restype = cwipc_source_p


    return _cwipc_realsense2_dll_reference
        
def cwipc_realsense2():
    """Returns a cwipc_source object that grabs from a realsense2 camera and returns cwipc object on every get() call."""
    rv = _cwipc_realsense2_dll().cwipc_realsense2()
    return cwipc_source(rv)
     
def main():
    grabber = cwipc_realsense2()
    pc = grabber.get()
    if not pc:
        print('Could not read pointcloud from realsense2 grabber')
    points = pc.get_points()
    print('Pointcloud contained %d points' % len(points))
    
if __name__ == '__main__':
    main()
    
    
