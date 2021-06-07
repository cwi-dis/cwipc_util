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

class _NetDecoder(threading.Thread):
    
    QUEUE_WAIT_TIMEOUT=1
    
    
    def __init__(self, source, verbose=False):
        threading.Thread.__init__(self)
        self.source = source
        self.running = False
        self.verbose = verbose
        self.queue = queue.Queue()
        self.times_decode = []
        
    def free(self):
        pass
        
    def start(self):
        assert not self.running
        if self.verbose: print('netdecoder: start')
        self.running = True
        threading.Thread.start(self)
        if hasattr(self.source, 'start'):
            self.source.start()
        
    def stop(self):
        if self.verbose: print('netdecoder: stop')
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
        if self.verbose: print(f"netdecoder: thread started")
        while self.running:
            if not self.source.available(True):
                continue
            cpc = self.source.get()
            if not cpc:
                print(f'netdecoder: source.get returned no data')
                self.queue.put(None)
                break
            t1 = time.time()
            pc = self._decompress(cpc)
            t2 = time.time()
            self.times_decode.append(t2-t1)
            self.queue.put(pc)
            if self.verbose: print(f'netdecoder: decoded pointcloud with {pc.count()} points')
        if self.verbose: print(f"netdecoder: thread exiting")

    def _decompress(self, cpc):
        decomp = cwipc.codec.cwipc_new_decoder()
        decomp.feed(cpc)
        gotData = decomp.available(True)
        if not gotData: return None
        pc = decomp.get()
        return pc

    def statistics(self):
        self.print1stat('decodetime', self.times_decode)
        if hasattr(self.source, 'statistics'):
            self.source.statistics()
        
    def print1stat(self, name, values, isInt=False):
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
    
def cwipc_source_decoder(source, verbose=False):
    """Return cwipc_source-like object that reads compressed pointclouds from another source and decompresses them"""
    if cwipc.codec == None:
        raise RuntimeError("netdecoder requires cwipc.codec which is not available")
    rv = _NetDecoder(source, verbose=verbose)
    return rv
        
