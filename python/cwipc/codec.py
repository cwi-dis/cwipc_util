import ctypes
import ctypes.util
from .util import CwipcError, cwipc, cwipc_source, cwipc_point, cwipc_point_array
from .util import cwipc_p, cwipc_source_p

__all__ = [
    "cwpic_decompress"
]
    
_cwipc_codec_dll_reference = None

#
# NOTE: the signatures here must match those in cwipc_util/api.h or all hell will break loose
#
def _cwipc_codec_dll(libname=None):
    """Load the cwipc_util DLL and assign the signatures (if not already loaded)"""
    global _cwipc_codec_dll_reference
    if _cwipc_codec_dll_reference: return _cwipc_codec_dll_reference
    
    if libname == None:
        libname = ctypes.util.find_library('cwipc_codec')
        if not libname:
            raise RuntimeError('Dynamic library cwipc_util not found')
    assert libname
    _cwipc_codec_dll_reference = ctypes.CDLL(libname)
    
    _cwipc_codec_dll_reference.cwipc_decompress.argtypes = [ctypes.c_void_p, ctypes.c_int]
    _cwipc_codec_dll_reference.cwipc_decompress.restype = cwipc_p


    return _cwipc_codec_dll_reference
        
def cwipc_decompress(data):
    """Decompress a compressed pointcloud. Returns cwipc."""
    length = len(data)
    rv = _cwipc_codec_dll().cwipc_decompress(ctypes.cast(data, ctypes.c_void_p), length)
    if rv:
        return cwipc(rv)
    return None
    
def main():
    import sys
    if len(sys.argv) != 2:
        print('Usage: %s cwipcfile' % sys.argv[0])
        sys.exit(1)
    data = open(sys.argv[1], 'rb').read()
    pc = cwipc_decompress(data)
    if not pc:
        print('Could not read pointcloud from %s' % sys.argv[1])
    points = pc.get_points()
    print('File contained %d points' % len(points))
    
if __name__ == '__main__':
    main()
    
    
