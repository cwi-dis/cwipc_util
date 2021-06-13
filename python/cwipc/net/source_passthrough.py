import time
import os
import socket
import threading
import queue
import cwipc

try:
    import cwipc.codec
except ModuleNotFoundError:
    cwipc.codec = None

class _NetPassthrough(threading.Thread):
    
    QUEUE_WAIT_TIMEOUT=1
    
    
    def __init__(self, source, verbose=False):
        threading.Thread.__init__(self)
        self.source = source
        self.running = False
        self.verbose = verbose
        self.queue = queue.Queue()
        
    def free(self):
        pass
        
    def start(self):
        assert not self.running
        if self.verbose: print('passthrough: start')
        self.running = True
        threading.Thread.start(self)
        if hasattr(self.source, 'start'):
            self.source.start()
        
    def stop(self):
        if self.verbose: print('passthrough: stop')
        self.running = False
        if hasattr(self.source, 'stop'):
            self.source.stop()
        self.queue.put(None)
        self.join()
        
    def eof(self):
        return not self.running or self.queue.empty() and self.source.eof()
    
    def available(self, wait=False):
        # xxxjack if wait==True should get and put
        if not self.running:
            return False
        if not self.queue.empty():
            return True
        return self.source.available(wait)
        
    def get(self):
        if self.eof():
            return None
        pc = self.queue.get()
        return pc

    def run(self):
        if self.verbose: print(f"passthrough: thread started")
        while self.running:
            if not self.source.available(True):
                continue
            cpc = self.source.get()
            if not cpc:
                print(f'passthrough: source.get returned no data')
                self.queue.put(None)
                break
            pc = cwipc.cwipc_from_packet(cpc)
            self.queue.put(pc)
            if self.verbose: print(f'passthrough: decoded pointcloud with {pc.count()} points')
        if self.verbose: print(f"passthrough: thread exiting")

    def _decompress(self, cpc):
        decomp = cwipc.codec.cwipc_new_decoder()
        decomp.feed(cpc)
        gotData = decomp.available(True)
        if not gotData: return None
        pc = decomp.get()
        return pc

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
    
def cwipc_source_passthrough(source, verbose=False):
    """Return cwipc_source-like object that reads serialized (uncompressed) pointclouds from another source and returns them"""
    rv = _NetPassthrough(source, verbose=verbose)
    return rv
        
