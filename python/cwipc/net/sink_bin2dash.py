import ctypes
import ctypes.util
import time
import os

_bin2dash_dll_reference = None

BIN2DASH_API_VERSION = 0x20200327A

class Bin2dashError(RuntimeError):
    pass

def VRT_4CC(code):
    """Convert anything reasonable (bytes, string, int) to 4cc integer"""
    if isinstance(code, int):
        return code
    if not isinstance(code, bytes):
        assert isinstance(code, str)
        code = code.encode('ascii')
    assert len(code) == 4
    rv = (code[0]<<24) | (code[1]<<16) | (code[2]<<8) | (code[3])
    return rv
    
def repr_4CC(code):
    bstr = b''
    
    
class vrt_handle_p(ctypes.c_void_p):
    pass
    
class FrameInfo(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_longlong)
    ]
    
class streamDesc(ctypes.Structure):
    _fields_ = [
        ("MP4_4CC", ctypes.c_uint32),
        ("objectX", ctypes.c_uint32),
        ("objectY", ctypes.c_uint32),
        ("objectWidth", ctypes.c_uint32),
        ("objectHeight", ctypes.c_uint32),
        ("totalWidth", ctypes.c_uint32),
        ("totalHeight", ctypes.c_uint32),
    ]

    def __init__(self, fourcc, *args):
        super(streamDesc, self).__init__(VRT_4CC(fourcc), *args)
    
def _bin2dash_dll(libname=None):
    global _bin2dash_dll_reference
    if _bin2dash_dll_reference: return _bin2dash_dll_reference
    
    if libname == None:
        libname = os.environ.get('VRTOGETHER_BIN2DASH_PATH')
        if not libname:
            libname = ctypes.util.find_library('bin2dash')
            if not libname:
                libname = ctypes.util.find_library('bin2dash.so')
            if not libname:
                raise Bin2dashError('Dynamic library bin2dash not found')
    assert libname
    # Signals library needs to be able to find some data files stored next to the DLL.
    # Tell it where they are.
    if os.path.isabs(libname) and not 'SIGNALS_SMD_PATH' in os.environ:
        libdirname = os.path.dirname(libname)
        os.putenv('SIGNALS_SMD_PATH', libdirname)
    _bin2dash_dll_reference = ctypes.cdll.LoadLibrary(libname)
    
    _bin2dash_dll_reference.vrt_create_ext.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(streamDesc), ctypes.c_char_p, ctypes.c_int, ctypes.c_int, ctypes.c_uint64]
    _bin2dash_dll_reference.vrt_create_ext.restype = vrt_handle_p
    
    _bin2dash_dll_reference.vrt_create.argtypes = [ctypes.c_char_p, ctypes.c_uint, ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
    _bin2dash_dll_reference.vrt_create.restype = vrt_handle_p
    
    _bin2dash_dll_reference.vrt_destroy.argtypes = [vrt_handle_p]
    _bin2dash_dll_reference.vrt_destroy.restype = None
    
    _bin2dash_dll_reference.vrt_push_buffer_ext.argtypes = [vrt_handle_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_size_t]
    _bin2dash_dll_reference.vrt_push_buffer_ext.restype = ctypes.c_bool
    
    _bin2dash_dll_reference.vrt_push_buffer.argtypes = [vrt_handle_p, ctypes.c_char_p, ctypes.c_size_t]
    _bin2dash_dll_reference.vrt_push_buffer.restype = ctypes.c_bool
    
    _bin2dash_dll_reference.vrt_get_media_time_ext.argtypes = [vrt_handle_p, ctypes.c_int, ctypes.c_int]
    _bin2dash_dll_reference.vrt_get_media_time_ext.restype = ctypes.c_int64
    
    _bin2dash_dll_reference.vrt_get_media_time.argtypes = [vrt_handle_p, ctypes.c_int]
    _bin2dash_dll_reference.vrt_get_media_time.restype = ctypes.c_int64
    
    return _bin2dash_dll_reference
 
class _CpcBin2dashSink:
    def __init__(self, url="", *, verbose=False, nodrop=False, streamDescs=None, fourcc=None, seg_dur_in_ms=None, timeshift_buffer_depth_in_ms=None):
        if seg_dur_in_ms == None: seg_dur_in_ms=10000
        if timeshift_buffer_depth_in_ms == None: timeshift_buffer_depth_in_ms=30000
        self.verbose = verbose
        self.nodrop = nodrop
        self.producer = None
        self.url = url
        self.dll = None
        self.handle = None
        self.dll = _bin2dash_dll()
        assert self.dll
        url = url.encode('utf8')
        if streamDescs != None:
            assert fourcc == None   # Can only use fourcc or streamDescs
            streamDescCount = len(streamDescs)
            # ctypes array constructors are a bit weird. Check the documentation.
            c_streamDescs = (streamDesc*streamDescCount)(*streamDescs)
            if self.verbose:
                for i in range(streamDescCount):
                    print(f"bin2dash: streamDesc[{i}]: MP4_4CC={c_streamDescs[i].MP4_4CC.to_bytes(4, 'big')}={c_streamDescs[i].MP4_4CC}, objectX={c_streamDescs[i].objectX}, objectY={c_streamDescs[i].objectY}, objectWidth={c_streamDescs[i].objectWidth}, objectHeight={c_streamDescs[i].objectHeight}, totalWidth={c_streamDescs[i].totalWidth}, totalHeight={c_streamDescs[i].totalHeight}")
            self.handle = self.dll.vrt_create_ext("bin2dashSink".encode('utf8'), streamDescCount, c_streamDescs, url, seg_dur_in_ms, timeshift_buffer_depth_in_ms, BIN2DASH_API_VERSION)
            if not self.handle:
                raise Bin2dashError(f"vrt_create_ext({url}) failed")
        else:
            if fourcc == None:
                fourcc = VRT_4CC("cwi1")
            else:
                fourcc = VRT_4CC(fourcc)
            self.handle = self.dll.vrt_create("bin2dashSink".encode('utf8'), fourcc, url, seg_dur_in_ms, timeshift_buffer_depth_in_ms)
            if not self.handle:
                raise Bin2dashError(f"vrt_create({url}) failed")
        assert self.handle
        
    def __del__(self):
        self.free()
        
    def start(self):
        pass
        
    def stop(self):
        pass
        
    def set_producer(self, producer):
        self.producer = None
        
    def free(self):
        if self.handle:
            assert self.dll
            self.dll.vrt_destroy(self.handle)
            self.handle = None
            
    def feed(self, buffer, stream_index=None):
        if not self.handle:
            return False
        assert self.dll
        length = len(buffer)
        if stream_index == None:
            ok = self.dll.vrt_push_buffer(self.handle, bytes(buffer), length)
        else:
            ok = self.dll.vrt_push_buffer_ext(self.handle, stream_index, bytes(buffer), length)
        if not ok:
            raise Bin2dashError(f"vrt_push_buffer(..., {length}) failed")
        return ok

    def canfeed(self, timestamp, wait=True):
        return not not self.handle

def cwipc_sink_bin2dash(url, verbose=False, nodrop=False, **kwargs):
    return _CpcBin2dashSink(url, verbose=verbose, nodrop=nodrop, **kwargs)