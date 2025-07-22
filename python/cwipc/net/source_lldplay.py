import ctypes
import ctypes.util
import time
import os
import sys
import threading
import queue
from typing import Optional, List, Union
from .abstract import cwipc_rawsource_abstract, cwipc_source_abstract, cwipc_abstract, vrt_fourcc_type

LLDASH_PLAYOUT_API_VERSION = 0x20250722

_lldplay_dll_reference = None

class LLDashPlayoutError(RuntimeError):
    pass
    
class lldplay_handle_p(ctypes.c_void_p):
    pass
    
class FrameInfo(ctypes.Structure):
    timestamp : int

    _fields_ = [
        ("timestamp", ctypes.c_longlong)
    ]
    
class streamDesc(ctypes.Structure):
    MP4_4CC : int
    tileNumber : int
    x : int
    y : int
    z : int
    totalWidth : int
    totalHeight : int

    _fields_ = [
        ("MP4_4CC", ctypes.c_uint32),
        ("tileNumber", ctypes.c_uint32), # official DASH: objectX. Re-targeted for pointclouds
        ("x", ctypes.c_uint32), # official DASH: objectY. Re-targeted for pointclouds
        ("y", ctypes.c_uint32), # Official DASH: objectWidth. Retargeted for pointclouds
        ("z", ctypes.c_uint32), # Official DASH: objectHeight. Retargeted for pointclouds
        ("totalWidth", ctypes.c_uint32),
        ("totalHeight", ctypes.c_uint32),
    ]
streamDesc_pythonic = tuple[int, int, int, int, int, int, int]
tileInfo_pythonic = tuple[int, int, tuple[int, int, int], int]
      
LLDashPlayoutErrorCallbackType = ctypes.CFUNCTYPE(None, ctypes.c_char_p, ctypes.c_int)
    
def _lldplay_dll(libname : Optional[str]=None) -> ctypes.CDLL:
    global _lldplay_dll_reference
    if _lldplay_dll_reference: return _lldplay_dll_reference
    
    if libname == None:
        # Current preferred use: SIGNALS_SMD_PATH points to the right directory.
        dirname = os.environ.get('SIGNALS_SMD_PATH')
        if dirname:
            libname = os.path.join(dirname, 'lldash_play.so')
        if not libname:
            libname = ctypes.util.find_library('lldash_play.so')
            if not libname:
                libname = ctypes.util.find_library('lldash_play')
            if not libname:
                raise LLDashPlayoutError('Dynamic library lldash_play not found')
    assert libname
    # Signals library needs to be able to find some data files stored next to the DLL.
    # Tell it where they are.
    if os.path.isabs(libname) and not 'SIGNALS_SMD_PATH' in os.environ:
        libdirname = os.path.dirname(libname)
        os.putenv('SIGNALS_SMD_PATH', libdirname)
    _lldplay_dll_reference = ctypes.cdll.LoadLibrary(libname)
    
    _lldplay_dll_reference.lldplay_create.argtypes = [ctypes.c_char_p, LLDashPlayoutErrorCallbackType, ctypes.c_int, ctypes.c_uint64]
    _lldplay_dll_reference.lldplay_create.restype = lldplay_handle_p
    
    _lldplay_dll_reference.lldplay_destroy.argtypes = [lldplay_handle_p]
    _lldplay_dll_reference.lldplay_destroy.restype = None
    
    _lldplay_dll_reference.lldplay_play.argtypes = [lldplay_handle_p, ctypes.c_char_p]
    _lldplay_dll_reference.lldplay_play.restype = ctypes.c_bool
    
    _lldplay_dll_reference.lldplay_get_stream_count.argtypes = [lldplay_handle_p]
    _lldplay_dll_reference.lldplay_get_stream_count.restype = ctypes.c_int
    
    _lldplay_dll_reference.lldplay_get_stream_info.argtypes = [lldplay_handle_p, ctypes.c_int, ctypes.POINTER(streamDesc)]
    _lldplay_dll_reference.lldplay_get_stream_info.restype = ctypes.c_bool
    
    _lldplay_dll_reference.lldplay_enable_stream.argtypes = [lldplay_handle_p, ctypes.c_int, ctypes.c_int]
    _lldplay_dll_reference.lldplay_enable_stream.restype = ctypes.c_bool
    
    _lldplay_dll_reference.lldplay_disable_stream.argtypes = [lldplay_handle_p, ctypes.c_int]
    _lldplay_dll_reference.lldplay_disable_stream.restype = ctypes.c_bool
    
    _lldplay_dll_reference.lldplay_grab_frame.argtypes = [lldplay_handle_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p]
    _lldplay_dll_reference.lldplay_grab_frame.restype = ctypes.c_size_t

    _lldplay_dll_reference.lldplay_get_version.argtypes = []
    _lldplay_dll_reference.lldplay_get_version.restype = ctypes.c_char_p
    
    return _lldplay_dll_reference
 
class _LLDashPlayoutSource(threading.Thread, cwipc_rawsource_abstract):
    # xxxjack need to check that these are still needed...
    # If no data is available from the sub this is how long we sleep before trying again:
    SUB_WAIT_TIME=0.01
    # If no data is available from the sub for this long we treat it as end-of-file:
    SUB_EOF_TIME=10
    # How long to wait for data to be put into the queue (really only to forestall deadlocks on close)
    QUEUE_WAIT_TIMEOUT=1
    # Should we check all streams for available data, or only ones where we are expecting it?
    EAGER_RECEIVE = True
    # Should we forward all available streams?
    EAGER_FORWARD = True

    handle : Optional[lldplay_handle_p]
    dll : ctypes.CDLL
    tile_info : Optional[List[tileInfo_pythonic]]
    times_receive: List[float]
    sizes_receive: List[int]
    bandwidth_receive: List[float]
    unwanted_receive: List[int]
    fourcc : Optional[vrt_fourcc_type]

    output_queue : queue.Queue[Optional[bytes]]
    popped_queue_head : Optional[bytes]
    queue_lock : threading.Lock

    def __init__(self, url : str, streamIndex : int=0, verbose : bool=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._LLDashPlayoutSource'
        self.verbose = verbose
        self.url = url
        self.handle = None
        self.started = False
        self.running = False
        self.failed = False
        self.output_queue = queue.Queue(maxsize=2)
        self.popped_queue_head = None
        self.queue_lock = threading.Lock()
        self.times_receive = []
        self.sizes_receive = []
        self.bandwidths_receive = []
        self.unwanted_receive = []
        self.streamIndex = streamIndex
        self.streamCount = 0
        self.tile_info = None
        self.fourcc = None
        self.error_condition = False
        lldash_log_setting = os.environ.get("LLDASH_LOGGING", None)
        self.lldash_logging = not not lldash_log_setting
        self.dll = _lldplay_dll()
        assert self.dll
        if self.verbose:
            print(f"lldash_play: native library version: {self.dll.lldplay_get_version().decode('utf8')}", file=sys.stderr, flush=True)
        self.lldash_log(event="source_lldplay_init", version=self.dll.lldplay_get_version().decode('utf8'))
        if self.verbose: print(f"lldash_play: lldplay_create()")
        self._onLLDashPlayoutError = LLDashPlayoutErrorCallbackType(self._onLLDashPlayoutError)
        msgLevel = 3 if self.verbose else 0
        self.handle = self.dll.lldplay_create("cwipc_lldplay".encode('utf8'), self._onLLDashPlayoutError, msgLevel, LLDASH_PLAYOUT_API_VERSION)
        if not self.handle:
            raise LLDashPlayoutError("lldplay_create failed")
        
    def _onLLDashPlayoutError(self, cmsg : bytes, level : int):
        """Callback function passed to lldplay_create: preint (or re-raise) LLDashPlayout errors in Python environment"""
        msg = cmsg.decode('utf8')
        levelName = {0:"error", 1:"warning", 2:"info message", 3:"debug message"}.get(level, f"level-{level} message")
    
        print(f"lldash_play: asynchronous {levelName}: {msg}", file=sys.stderr, flush=True)
        self.lldash_log(event="async_log_message", level=level, msg=msg)
        # xxxjack raising an exception from a ctypes callback doesn't work.
        # Need to fix at some point.
        if level == 0:
            self.error_condition = True
            # raise LLDashPlayoutError(msg)
        
    def lldash_log(self, **kwargs):
        if not self.lldash_logging:
            return
        lts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) + f".{int(time.time()*1000)%1000:03d}"
        message = f"lldash_play: ts={lts}"
        for k, v in kwargs.items():
            message += f" {k}={v}"
        print(message)
        sys.stdout.flush()
        
    def __del__(self):
        self.free()
        
    def free(self) -> None:
        if self.handle:
            assert self.dll
            if self.verbose: print(f"lldash_play: calling lldplay_destroy()")
            self.dll.lldplay_destroy(self.handle)
            if self.verbose: print(f"lldash_play: lldplay_destroy() returned")
            self.handle = None
    
    def set_fourcc(self, fourcc : vrt_fourcc_type) -> None:
        self.fourcc = fourcc
        # xxxjack we should check that the fourcc is valid for the stream
        # but we don't do so yet.

    def start(self) -> None:
        assert self.handle
        assert self.dll
        if self.verbose: print(f"lldash_play: lldplay_play({self.url})")
        self.lldash_log(event="lldplay_play_call", url=self.url)
        ok = self.dll.lldplay_play(self.handle, self.url.encode('utf8'))
        self.lldash_log(event="lldplay_play_return", ok=ok)
        if not ok:
            self.failed = True
            self.output_queue.put(None)
            raise LLDashPlayoutError("lldash_play: lldplay_play returned false")
        nstreams = self.dll.lldplay_get_stream_count(self.handle)
        if self.verbose: print(f"lldash_play: lldplay_get_stream_count() -> {nstreams}")
        assert nstreams > self.streamIndex
        self.running = True
        self.started = True
        if True and self.verbose:
            for i in range(nstreams):
                srd_info = self._srd_info_for_stream(i)
                print(f"lldash_play: stream[{i}]: {srd_info}")
        threading.Thread.start(self)
        
    def stop(self) -> None:
        self.running = False
        self.output_queue.put(None)
        if self.started:
            self.join()
        
    def eof(self) -> bool:
        if self.error_condition:
            return False
        return self.failed or (self.started and self.output_queue.empty() and not self.running)
    
    def available(self, wait : bool=False) -> bool:
        if self.error_condition:
            return False
        try:
            with self.queue_lock:
                if self.popped_queue_head:
                    return True
                if not self.output_queue.empty():
                    return True
                if not wait or not self.running or self.failed:
                    return False
                try:
                    packet = self.output_queue.get(timeout=self.QUEUE_WAIT_TIMEOUT)
                    if packet:
                        assert not self.popped_queue_head
                        self.popped_queue_head = packet
                    return not not packet
                except queue.Empty:
                    return False
        finally:
            time.sleep(0.0001)  # Give other threads a chance to run
                            
    def get(self) -> Optional[bytes]:
        with self.queue_lock:
            if self.popped_queue_head:
                rv = self.popped_queue_head
                self.popped_queue_head = None
                return rv
            if self.eof():
                return None
            packet = self.output_queue.get()
            return packet

    def count(self) -> int:
        if not self.streamCount:
            if self.error_condition:
                return 0
            assert self.handle
            assert self.dll
            assert self.started
            self.streamCount = self.dll.lldplay_get_stream_count(self.handle)
        return self.streamCount

    def maxtile(self) -> int:
        self._init_tile_info()
        assert self.tile_info != None
        return len(self.tile_info)
    
    def get_tileinfo_dict(self, tilenum : int):
        """Return tile information for tile tilenum as Python dictionary"""
        self._init_tile_info()
        assert self.tile_info != None
        info = self.tile_info[tilenum]
        mp4_4cc, tileNumber, (x, y, z), qualityCount = info
        normal = dict(x=(x/1000.0), y=(y/1000.0), z=(z/1000.0))
        # Computing ncamera is difficult...
        return dict(normal=normal, cameraName=f"tile-{tilenum}", cameraMask=tileNumber, nquality=qualityCount, mp4_4cc=mp4_4cc)

    def _srd_info_for_stream(self, num : int) -> streamDesc_pythonic:
        assert self.handle
        assert self.dll
        assert self.started
        c_desc = streamDesc()
        ok = self.dll.lldplay_get_stream_info(self.handle, num, c_desc)
        return (c_desc.MP4_4CC, c_desc.tileNumber, c_desc.x, c_desc.y, c_desc.z, c_desc.totalWidth, c_desc.totalHeight)

    def _init_tile_info(self) -> List[tileInfo_pythonic]:
        if self.tile_info:
            return self.tile_info
        streamdesc_to_streamcount : dict[streamDesc_pythonic, int] = {}
        ordered_tiles = []
        for i in range(self.count()):
            streamDesc = self._srd_info_for_stream(i)
            if not streamDesc in streamdesc_to_streamcount:
                streamdesc_to_streamcount[streamDesc] = 1
                ordered_tiles.append(streamDesc)
            else:
                streamdesc_to_streamcount[streamDesc] += 1
        self.tile_info = []
        for tileDesc in ordered_tiles:
            mp4_4cc, tileNumber, x, y, z, totalWidth, totalHeight = tileDesc
            qualityCount = streamdesc_to_streamcount[tileDesc]
            self.tile_info.append((mp4_4cc, tileNumber, (x, y, z), qualityCount))
        return self.tile_info

    def enable_stream(self, tileNum : int, qualityNum : int) -> bool:
        assert self.handle
        assert self.dll
        assert self.started
        if self.error_condition:
            return False
        if self.verbose: print(f"lldash_play: lldplay_enable_stream(handle, {tileNum}, {qualityNum})")
        self.lldash_log(event="lldplay_enable_stream", tileNum=tileNum, qualityNum=qualityNum, url=self.url)
        ok = self.dll.lldplay_enable_stream(self.handle, tileNum, qualityNum)
        if not ok:
            return False
        if tileNum != 0:
            raise NotImplementedError
        self.streamIndex = qualityNum
        return True
        
    def disable_stream(self, tileNum : int) -> bool:
        assert self.handle
        assert self.dll
        assert self.started
        if self.error_condition:
            return False
        if self.verbose: print(f"lldash_play: lldplay_disable_stream(handle, {tileNum})")
        self.lldash_log(event="lldplay_disable_stream", tileNum=tileNum, url=self.url)
        rv = self.dll.lldplay_disable_stream(self.handle, tileNum)
        return rv
        
    def run(self) -> None:
        if self.verbose: print(f"lldash_play: thread started")
        last_successful_read_time = time.time()
        try:
            while self.running and not self.error_condition:
                receivedAnything = False
                if self.EAGER_RECEIVE:
                    streamsToCheck = range(self.count())
                else:
                    streamsToCheck = [self.streamIndex]
                if self.EAGER_FORWARD:
                    streamsToForward = set(range(self.count()))
                else:
                    streamsToForward = set([self.streamIndex])
                for streamIndex in streamsToCheck:
                    #if self.verbose: print(f"lldash_play: read: lldplay_grab_frame(handle, {streamIndex}, None, 0, None)")
                    length = self.dll.lldplay_grab_frame(self.handle, streamIndex, None, 0, None)
                    #if self.verbose: print(f"lldash_play: read: lldplay_grab_frame(handle, {streamIndex}, None, 0, None) -> {length}")

                    if length == 0:
                        continue
            
                    packet = bytearray(length)
                    ptr_char = (ctypes.c_char * length).from_buffer(packet)
                    ptr = ctypes.cast(ptr_char, ctypes.c_void_p)
                    if self.verbose: print(f"lldash_play: read: lldplay_grab_frame(handle, {streamIndex}, ptr, {length}, None)")
                    length2 = self.dll.lldplay_grab_frame(self.handle, streamIndex, ptr, length, None)
                    self.lldash_log(event="lldplay_grab_frame_returned", streamIndex=streamIndex, length=length, url=self.url)
                    if length2 != length:
                        raise LLDashPlayoutError("read_cpc(stream={streamIndex}: was promised {length} bytes but got only {length2})")
                    
                    receivedAnything = True
                    
                    if not streamIndex in streamsToForward:
                        print(f'lldash_play: drop {length2} received on stream {streamIndex}')
                        self.unwanted_receive.append(length2)
                        continue
                        
                    now = time.time()
                    delta = now-last_successful_read_time
                    last_successful_read_time = now
                    if delta <= 0:
                        # sub-millisecond read. Guess.
                        delta = 0.0005

                    self.times_receive.append(delta)
                    self.sizes_receive.append(length2)
                    self.bandwidths_receive.append(len(packet)/delta)
                    self.output_queue.put(packet)

                if not receivedAnything:
                    if time.time() - last_successful_read_time > self.SUB_EOF_TIME:
                        # Too long with no data, assume the producer has exited and treat as end of file
                        print(f'lldash_play: nothing received for {self.SUB_EOF_TIME} seconds, assuming end of file')
                        break
                    # We wait a while and try again (if not closed)
                    time.sleep(self.SUB_WAIT_TIME)
        finally:
            self.running = False
            self.output_queue.put(None)
        if self.verbose: print(f"lldash_play: thread exiting")
        
    def statistics(self):
        self.print1stat('receive_duration', self.times_receive)
        self.print1stat('packetsize', self.sizes_receive, isInt=True)
        self.print1stat('bandwidth', self.bandwidths_receive)
        self.print1stat('unwanted_packetsize', self.unwanted_receive, isInt=True)
        
    def print1stat(self, name, values : Union[List[int], List[float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print('lldash_play: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'lldash_play: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'lldash_play: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))
        
def cwipc_source_lldplay(address : str, verbose=False) -> cwipc_rawsource_abstract:
    """Return cwipc_source-like object that reads compressed pointclouds from a DASH stream using MotionSpell lldash"""
    _lldplay_dll()
    src = _LLDashPlayoutSource(address, verbose=verbose)
    return src
