import time
import os
import socket
import threading
import queue
import cwipc
from typing import Optional
from abstract import cwipc_rawsource_abstract, cwipc_source_abstract, cwipc_abstract

try:
    import cwipc.codec
except ModuleNotFoundError:
    cwipc.codec = None

class _NetPassthrough(threading.Thread, cwipc_source_abstract):
    
    QUEUE_WAIT_TIMEOUT=1
    
    source : cwipc_rawsource_abstract
    _queue : queue.Queue[Optional[cwipc_abstract]]

    def __init__(self, source : cwipc_rawsource_abstract, verbose : bool=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._NetPassthrough'
        self.source = source
        self.running = False
        self.verbose = verbose
        self._queue = queue.Queue()
        
    def free(self) -> None:
        pass
        
    def start(self) -> None:
        assert not self.running
        if self.verbose: print('passthrough: start')
        self.running = True
        threading.Thread.start(self)
        if hasattr(self.source, 'start'):
            self.source.start()
        
    def stop(self) -> None:
        if self.verbose: print('passthrough: stop')
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

    def run(self) -> None:
        if self.verbose: print(f"passthrough: thread started")
        while self.running:
            if not self.source.available(True):
                continue
            cpc = self.source.get()
            if not cpc:
                print(f'passthrough: source.get returned no data')
                self._queue.put(None)
                break
            pc = cwipc.cwipc_from_packet(cpc)
            self._queue.put(pc)
            if self.verbose: print(f'passthrough: decoded pointcloud with {pc.count()} points')
        if self.verbose: print(f"passthrough: thread exiting")

    def statistics(self):
        if hasattr(self.source, 'statistics'):
            self.source.statistics()
        
    def print1stat(self, name, values, isInt=False):
        count = len(values)
        if count == 0:
            print('passthrough: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'passthrough: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'passthrough: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

    def request_auxiliary_data(self, name: str) -> None:
        assert False

    def auxiliary_data_requested(self, name: str) -> bool:
        return False
    
def cwipc_source_passthrough(source, verbose=False):
    """Return cwipc_source-like object that reads serialized (uncompressed) pointclouds from another source and returns them"""
    rv = _NetPassthrough(source, verbose=verbose)
    return rv
        
