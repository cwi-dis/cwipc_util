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

class _NetClientSource(threading.Thread):
    
    QUEUE_WAIT_TIMEOUT=1
    verbose=False
    
    def __init__(self, address):
        threading.Thread.__init__(self)
        hostname, port = address.split(':')
        if not hostname:
            hostname = 'localhost'
        if not port:
            port = 4303
        port = int(port)
        self.hostname = hostname
        self.port = port
        self.running = False
        self._conn_refused = False
        self.queue = queue.Queue()
        
    def free(self):
        pass
        
    def start(self):
        assert not self.running
        self.running = True
        threading.Thread.start(self)
        
    def stop(self):
        self.running = False
        self.join()
        
    def eof(self):
        return self.queue.empty() and self._conn_refused
    
    def available(self, wait=False):
        # xxxjack if wait==True should get and put
        if not self.queue.empty():
            return True
        if not wait:
            return False
        try:
            pc = self.queue.get(timeout=self.QUEUE_WAIT_TIMEOUT)
            if pc:
                self.queue.put(pc)
            return not not pc
        except queue.Empty:
            return False
        
    def get(self):
        if self.eof():
            return None
        pc = self.queue.get()
        return pc

    def run(self):
        if self.verbose: print(f"netclient: thread started")
        while self.running and not self._conn_refused:
            with socket.socket() as s:
                try:
                    other = s.connect((self.hostname, self.port))
                except ConnectionRefusedError:
                    if self.verbose: print(f"netclient: connection refused")
                    self._conn_refused = True
                    break
                except socket.error as err:
                    print(f'netclient: connecting to {self.hostname}:{self.port}: {err}')
                    raise
                if self.verbose: print(f'netclient: connected')
                packet = b''
                while True:
                    data = s.recv(8192)
                    if not data: break
                    packet += data
                if self.verbose: print(f'netclient: received {len(packet)} bytes')
                pc = self._decompress(packet)
                self.queue.put(pc)
        if self.verbose: print(f"netclient: thread exiting")

    def _decompress(self, cpc):
        decomp = cwipc.codec.cwipc_new_decoder()
        decomp.feed(cpc)
        gotData = decomp.available(True)
        if not gotData: return None
        pc = decomp.get()
        return pc
    
def cwipc_netclient(address):
    """Return cwipc_source-like object that reads individual compressed pointclouds from a TCP-based server specified as host:port"""
    if cwipc.codec == None:
        raise RuntimeError("netclient requires cwipc.codec which is not available")
    source = _NetClientSource(address)
    return source
        
