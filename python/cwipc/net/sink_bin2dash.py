import ctypes
import ctypes.util
import sys
import os
import time
from typing import Optional, Any, List, Union
from .abstract import cwipc_producer_abstract, vrt_fourcc_type, VRT_4CC, cwipc_rawsink_abstract

_bin2dash_dll_reference = None

BIN2DASH_API_VERSION = 0x20200327A

class Bin2dashError(RuntimeError):
    pass

class vrt_handle_p(ctypes.c_void_p):
    pass
    
class FrameInfo(ctypes.Structure):
    _fields_ = [
        ("timestamp", ctypes.c_longlong)
    ]
    
class streamDesc(ctypes.Structure):
    _fields_ = [
        ("MP4_4CC", ctypes.c_uint32),
        ("tileNumber", ctypes.c_uint32), # official DASH: objectX. Re-targeted for pointclouds
        ("x", ctypes.c_uint32), # official DASH: objectY. Re-targeted for pointclouds
        ("y", ctypes.c_uint32), # Official DASH: objectWidth. Retargeted for pointclouds
        ("z", ctypes.c_uint32), # Official DASH: objectHeight. Retargeted for pointclouds
        ("totalWidth", ctypes.c_uint32),
        ("totalHeight", ctypes.c_uint32),
    ]

    def __init__(self, fourcc : vrt_fourcc_type, *args : Any):
        super(streamDesc, self).__init__(VRT_4CC(fourcc), *args)
    
def _bin2dash_dll(libname : Optional[str]=None) -> ctypes.CDLL:
    global _bin2dash_dll_reference
    if _bin2dash_dll_reference: return _bin2dash_dll_reference
    
    if libname == None:
        # Backward compatibility: if the environment variable VRTOGETHER_BIN2DASH_PATH is set, use that.
        libname = os.environ.get('VRTOGETHER_BIN2DASH_PATH')
        if not libname:
            # Current preferred use: SIGNALS_SMD_PATH points to the right directory.
            dirname = os.environ.get('SIGNALS_SMD_PATH')
            if dirname:
                libname = os.path.join(dirname, 'bin2dash.so')
        if not libname:
            libname = ctypes.util.find_library('bin2dash.so')
            if not libname:
                libname = ctypes.util.find_library('bin2dash')
            if not libname:
                raise Bin2dashError('Dynamic library bin2dash not found. Set SIGNALS_SMD_PATH to the directory containing the bin2dash library')
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
 
class _CpcBin2dashSink(cwipc_rawsink_abstract):
    """A DASH sink that streams multiple data streams to a MotionSpell DASH ingestion server.
    
    Uses the bin2dash native implementation under the hood.
    """
    
    streamDescs : Optional[List[streamDesc]]
    dll : ctypes.CDLL
    fourcc : Optional[vrt_fourcc_type]
    sizes_forward : List[int]

    def __init__(self, url : str="", *, verbose : bool=False, nodrop : bool=False, streamDescs : Optional[List[streamDesc]]=None, fourcc : Optional[vrt_fourcc_type]=None, seg_dur_in_ms : Optional[int]=None, timeshift_buffer_depth_in_ms : Optional[int]=None):
        if seg_dur_in_ms == None: seg_dur_in_ms=10000
        if timeshift_buffer_depth_in_ms == None: timeshift_buffer_depth_in_ms=30000
        self.verbose = verbose
        self.nodrop = nodrop
        self.url = url
        self.handle = None
        self.dll = _bin2dash_dll()
        assert self.dll
        self.url = url
        self.streamDescs = streamDescs
        self.fourcc = fourcc
        self.seg_dur_in_ms = seg_dur_in_ms
        self.timeshift_buffer_depth_in_ms = timeshift_buffer_depth_in_ms
        self.handle = None        
        self.sizes_forward = []
        self.bandwidths_forward = []
        lldash_log_setting = os.environ.get("LLDASH_LOGGING", None)
        self.lldash_logging = not not lldash_log_setting
        
    def __del__(self):
        self.free()
        
    def lldash_log(self, **kwargs):
        if not self.lldash_logging:
            return
        lts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) + f".{int(time.time()*1000)%1000:03d}"
        message = f"sink_bin2dash: ts={lts}"
        for k, v in kwargs.items():
            message += f" {k}={v}"
        print(message)
        sys.stdout.flush()
        
    def start(self) -> None:
        url = self.url.encode('utf8')
        if self.streamDescs != None:
            streamDescCount = len(self.streamDescs)
            # ctypes array constructors are a bit weird. Check the documentation.
            c_streamDescs = (streamDesc*streamDescCount)(*self.streamDescs)
            if self.verbose:
                for i in range(streamDescCount):
                    print(f"bin2dash: streamDesc[{i}]: MP4_4CC={c_streamDescs[i].MP4_4CC.to_bytes(4, 'big')}={c_streamDescs[i].MP4_4CC}, tileNumber={c_streamDescs[i].tileNumber}, x={c_streamDescs[i].x}, y={c_streamDescs[i].y}, z={c_streamDescs[i].z}, totalWidth={c_streamDescs[i].totalWidth}, totalHeight={c_streamDescs[i].totalHeight}")
            self.lldash_log(event="vrt_create_ext_call", url=self.url, seg_dur=self.seg_dur_in_ms, timeshift_buffer_depth=self.timeshift_buffer_depth_in_ms, streamDescCount=streamDescCount)
            self.handle = self.dll.vrt_create_ext("bin2dashSink".encode('utf8'), streamDescCount, c_streamDescs, url, self.seg_dur_in_ms, self.timeshift_buffer_depth_in_ms, BIN2DASH_API_VERSION)
            self.lldash_log(event="vrt_create_ext_returned", url=self.url)
            if not self.handle:
                raise Bin2dashError(f"vrt_create_ext({url}) failed")
        else:
            if self.fourcc == None:
                self.fourcc = VRT_4CC("cwi1")
            else:
                self.fourcc = VRT_4CC(self.fourcc)
            self.lldash_log(event="vrt_create_call", url=self.url, seg_dur=self.seg_dur_in_ms, timeshift_buffer_depth=self.timeshift_buffer_depth_in_ms)
            self.handle = self.dll.vrt_create("bin2dashSink".encode('utf8'), self.fourcc, url, self.seg_dur_in_ms, self.timeshift_buffer_depth_in_ms)
            self.lldash_log(event="vrt_create_returned", url=self.url)
            if not self.handle:
                raise Bin2dashError(f"vrt_create({url}) failed")
        assert self.handle
        
    def stop(self) -> None:
        pass
        
    def set_producer(self, producer : cwipc_producer_abstract) -> None:
        pass
        
    def set_fourcc(self, fourcc : vrt_fourcc_type) -> None:
        self.fourcc = fourcc
        
    def _set_streamDescs(self, streamDescs : List[streamDesc]) -> None:
        self.streamDescs = streamDescs
        
    def add_streamDesc(self, tilenum : int, x : Union[int, float], y : Union[int, float], z : Union[int, float]) -> int:
        """Specify that stream tilenum represents a tile with the given (x,y,z) orientation."""
        if not self.streamDescs:
            self.streamDescs = []
        if type(x) != int: x = int(x*1000)
        if type(y) != int: y = int(y*1000)
        if type(z) != int: z = int(z*1000)
        assert self.fourcc
        self.streamDescs.append(streamDesc(self.fourcc, tilenum, x, y, z))
        return len(self.streamDescs)-1
        
    def free(self) -> None:
        if self.handle:
            assert self.dll
            self.lldash_log(event="vrt_destroy_call", url=self.url)
            self.dll.vrt_destroy(self.handle)
            self.lldash_log(event="vrt_destroy_returned", url=self.url)
            self.handle = None
            
    def feed(self, buffer : Union[bytes, bytearray], stream_index : Optional[int]=None) -> bool:
        if not self.handle:
            return False
        assert self.dll
        length = len(buffer)
        ok : bool
        if stream_index == None:
            self.lldash_log(event="vrt_push_buffer_call", url=self.url, length=length)
            ok = self.dll.vrt_push_buffer(self.handle, bytes(buffer), length)
            self.lldash_log(event="vrt_push_buffer_returned", url=self.url)
        else:
            self.lldash_log(event="vrt_push_buffer_ext_call", url=self.url, stream_index=stream_index, length=length)
            ok = self.dll.vrt_push_buffer_ext(self.handle, stream_index, bytes(buffer), length)
            self.lldash_log(event="vrt_push_buffer_ext_returned", url=self.url)
            
        if ok:
            self.sizes_forward.append(length)
        else:
            raise Bin2dashError(f"vrt_push_buffer(handle, {stream_index}, buffer, {length}) failed")
        return ok

    def canfeed(self, timestamp : int, wait : bool=True) -> bool:
        return not not self.handle

    def statistics(self) -> None:
        self.print1stat('packetsize', self.sizes_forward)
        
    def print1stat(self, name : str, values : Union[List[int], List[float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print('bin2dash: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'bin2dash: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'bin2dash: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

def cwipc_sink_bin2dash(url : str, verbose : bool=False, nodrop : bool=False, **kwargs : Any) -> cwipc_rawsink_abstract:
    """Create a sink that transmits to a DASH ingestion server."""
    return _CpcBin2dashSink(url, verbose=verbose, nodrop=nodrop, **kwargs)