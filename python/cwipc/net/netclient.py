import time
import os
import socket
import cwipc
try:
    import cwipc.codec
except ModuleNotFoundError:
    cwipc.codec = None

class _NetClientSource:
    def __init__(self, address):
        hostname, port = address.split(':')
        if not hostname:
            hostname = 'localhost'
        if not port:
            port = 4303
        port = int(port)
        self.hostname = hostname
        self.port = port
        
    def free(self):
        pass
        
    def eof(self):
        return False
    
    def available(self, wait=False):
        return True
        
    def get(self):
        data = self._read_cpc()
        pc = self._decompress(data)
        return pc

    def _read_cpc(self):
        with socket.socket() as s:
            try:
                s.connect((self.hostname, self.port))
            except socket.error as err:
                print('connecting to {}:{}: {}'.format(self.hostname, self.port, err))
                raise
            rv = b''
            while True:
                data = s.recv(8192)
                if not data: break
                rv += data
            return rv

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
    return _NetClientSource(address)
        
