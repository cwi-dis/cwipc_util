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
    verbose=False
    
    def __init__(self, source):
        threading.Thread.__init__(self)
        self.source = source
        self.running = False
        self.queue = queue.Queue()
        
    def free(self):
        pass
        
    def start(self):
        assert not self.running
        self.running = True
        threading.Thread.start(self)
        if hasattr(self.source, 'start'):
            self.source.start()
        
    def stop(self):
        self.running = False
        if hasattr(self.source, 'stop'):
            self.source.stop()
        self.join()
        
    def eof(self):
        return self.queue.empty() and self.source.eof()
    
    def available(self, wait=False):
        # xxxjack if wait==True should get and put
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
            if cpc:
                pc = self._decompress(cpc)
                self.queue.put(pc)
        if self.verbose: print(f"netdecoder: thread exiting")

    def _decompress(self, cpc):
        decomp = cwipc.codec.cwipc_new_decoder()
        decomp.feed(cpc)
        gotData = decomp.available(True)
        if not gotData: return None
        pc = decomp.get()
        return pc
    
def cwipc_netdecoder(source):
    """Return cwipc_source-like object that reads individual compressed pointclouds from a TCP-based server specified as host:port"""
    if cwipc.codec == None:
        raise RuntimeError("netdecoder requires cwipc.codec which is not available")
    rv = _NetDecoder(source)
    return rv
        
