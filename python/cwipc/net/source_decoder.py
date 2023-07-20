import time
import os
import socket
import threading
import queue
from typing import Optional, List, Union
from abstract import cwipc_source_abstract, cwipc_abstract, cwipc_rawsource_abstract

try:
    from .. import codec
except ModuleNotFoundError:
    codec = None

class _NetDecoder(threading.Thread, cwipc_source_abstract):
    
    QUEUE_WAIT_TIMEOUT=1
    _queue : queue.Queue[Optional[cwipc_abstract]]

    def __init__(self, source, verbose : bool=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._NetDecoder'
        self.source = source
        self.running = False
        self.verbose = verbose
        self._queue = queue.Queue()
        self.times_decode = []
        self.streamNumber = None
        self._init_tiling()

    def _init_tiling(self) -> None:
        # Bit of a hack: export the select_stream() method if our source has multiple tiles/streams
        if hasattr(self.source, 'enable_stream'):
            self.select_stream = self._select_stream
            self.tileNum = 0
        
    def free(self) -> None:
        pass
        
    def start(self) -> None:
        assert not self.running
        if self.verbose: print('netdecoder: start', flush=True)
        self.running = True
        threading.Thread.start(self)
        if hasattr(self.source, 'start'):
            self.source.start()
        if hasattr(self.source, 'disable_stream'):
            # For sources with streams per tile disable all tiles except the first one.
            for i in range(1, self.source.maxtile()):
                print(f'netdecoder: disable tile {i}')
                self.source.disable_stream(i)
        
    def stop(self) -> None:
        if self.verbose: print('netdecoder: stop', flush=True)
        self.running = False
        if hasattr(self.source, 'stop'):
            self.source.stop()
        self._queue.put(None)
        self.join()
        
    def eof(self) -> bool:
        return not self.running or self._queue.empty() and self.source.eof()
    
    def available(self, wait : bool=False) -> bool:
        # xxxjack if wait==True should get and put
        if not self.running:
            return False
        if not self._queue.empty():
            return True
        return self.source.available(wait)
        
    def get(self) -> Optional[cwipc_abstract]:
        if self.eof():
            return None
        pc = self._queue.get()
        return pc

    def _select_stream(self, streamIndex : int) -> None:
        if self.verbose: print(f'netdecoder: select_stream({streamIndex}', flush=True)
        return self.source.enable_stream(self.tileNum, streamIndex)
        
    def run(self) -> None:
        if self.verbose: print(f"netdecoder: thread started", flush=True)
        while self.running:
            if self.source.eof():
                break
            cpc = self.source.get()
            if not cpc:
                print(f'netdecoder: source.get returned no data')
                continue
            t1 = time.time()
            pc = self._decompress(cpc)
            assert pc
            t2 = time.time()
            self.times_decode.append(t2-t1)
            self._queue.put(pc)
            if self.verbose: print(f'netdecoder: decoded pointcloud with {pc.count()} points', flush=True)
        if self.verbose: print(f"netdecoder: thread exiting", flush=True)

    def _decompress(self, cpc : bytes) -> Optional[cwipc_abstract]:
        assert codec
        decomp = codec.cwipc_new_decoder()
        decomp.feed(cpc)
        gotData = decomp.available(True)
        if not gotData: return None
        pc = decomp.get()
        return pc

    def statistics(self) -> None:
        self.print1stat('decodetime', self.times_decode)
        if hasattr(self.source, 'statistics'):
            self.source.statistics()
        
    def print1stat(self, name : str, values : List[Union[int, float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print('netdecoder: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'netdecoder: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'netdecoder: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

    def request_auxiliary_data(self, name: str) -> None:
        assert False

    def auxiliary_data_requested(self, name: str) -> bool:
        return False
    
def cwipc_source_decoder(source : cwipc_rawsource_abstract, verbose : bool=False) -> cwipc_source_abstract:
    """Return cwipc_source-like object that reads compressed pointclouds from another source and decompresses them"""
    if codec == None:
        raise RuntimeError("netdecoder requires cwipc.codec which is not available")
    rv = _NetDecoder(source, verbose=verbose)
    return rv
        
