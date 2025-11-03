import time
import os
import socket
import threading
import queue
import cwipc
import struct
from typing import Optional, Union, List, Tuple

from cwipc.net.abstract import vrt_fourcc_type
from .abstract import *

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

class _NetClientSource(threading.Thread, cwipc_rawsource_abstract):
    
    QUEUE_WAIT_TIMEOUT=1
    
    
    def __init__(self, address : str | Tuple[str, int], verbose=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._NetClientSource'
        if type(address) == str:
            hostname, port = address.split(':')
        else:
            hostname, port = address
        if not hostname:
            hostname = 'localhost'
        if not port:
            port = 4303
        port = int(port)
        self.hostname = hostname
        self.port = port
        self.switch_to_port : Optional[int] = None
        self.running = False
        self._conn_refused = False
        self.verbose = verbose
        self.verbose = True
        self.output_queue = queue.Queue(maxsize=2)
        self.times_receive = []
        self.sizes_receive = []
        self.bandwidths_receive = []
        self.fourcc : Optional[vrt_fourcc_type] = None
        
    def free(self):
        pass

    def switchport(self, port : int) -> None:
        if port != self.port:
            self.switch_to_port = port
            if self.verbose: print(f'netclient: will switch to port {port}')

    def set_fourcc(self, fourcc: vrt_fourcc_type) -> None:
        self.fourcc = fourcc
        
    def start(self):
        assert not self.running
        if self.verbose: print('netclient: start')
        self.running = True
        threading.Thread.start(self)
        
    def stop(self):
        if self.verbose: print('netclient: stop')
        self.running = False
        try:
            self.output_queue.put(None, block=False)
        except queue.Full:
            pass
        self.join()
        
    def eof(self):
        return self.output_queue.empty() and self._conn_refused
    
    def available(self, wait=False):
        if not self.output_queue.empty():
            return True
        if not wait or self._conn_refused:
            return False
        # Note: the following code may reorder packets...
        try:
            packet = self.output_queue.get(timeout=self.QUEUE_WAIT_TIMEOUT)
            if packet:
                self.output_queue.put(packet)
            return not not packet
        except queue.Empty:
            return False
        
    def get(self):
        if self.eof():
            return None
        packet = self.output_queue.get()
        return packet

    def run(self):
        if self.verbose: print(f"netclient: {self.port}: thread started")
        s : Optional[socket.socket] = None
        while self.running and not self._conn_refused:
            if self.switch_to_port != None:
                if s != None:
                    if self.verbose: print(f'netclient: disconnect from {s.getpeername()}')
                    s.close()
                s = None
                self.port = self.switch_to_port
                self.switch_to_port = None
            if s == None:
                if self.verbose: print(f'netclient: connecting to {self.hostname}:{self.port}')
                s = socket.socket()
                try:
                    other = s.connect((self.hostname, self.port))
                except ConnectionRefusedError:
                    if self.verbose: print(f"netclient: {self.port}: connection refused")
                    self._conn_refused = True
                    break
                except socket.error as err:
                    print(f'netclient: connecting to {self.hostname}:{self.port}: {err}')
                    raise
                if self.verbose: print(f'netclient: connected to {self.hostname}:{self.port}')
            t1 = time.time()
            hdr = s.recv(16, socket.MSG_WAITALL)
            if len(hdr) == 0:
                if self.verbose: print(f'netclient: eof, disconnect from {s.getpeername()}')
                s.close()
                s = None
                break
            if len(hdr) != 16:
                print(f"netclient: {self.port}: received short header ({len(hdr)} bytes in stead of 16)")
                if self.verbose: print(f'netclient: disconnect from {s.getpeername()}')
                s.close()
                s = None
                break
            assert len(hdr) == 16
            h_fourcc, h_length, h_timestamp = struct.unpack("=LLQ", hdr)
            if self.fourcc != None:
                assert VRT_4CC(self.fourcc) == h_fourcc
            data = s.recv(h_length, socket.MSG_WAITALL)
            if len(data) == 0:
                if self.verbose: print(f'netclient: eof, disconnect from {s.getpeername()}')
                s.close()
                s = None
                break
            if len(data) != h_length:
                print(f"netclient: {self.port}: received data header ({len(data)} bytes in stead of {h_length})")
                if self.verbose: print(f'netclient: bad length, disconnect from {s.getpeername()}')
                s.close()
                s = None
                break
            assert h_length == len(data)
            # Ignore h_timestamp for now.
            t2 = time.time()
            if t2 == t1: t2 = t1 + 0.0005
            self.times_receive.append(t2-t1)
            self.sizes_receive.append(len(data))
            self.bandwidths_receive.append(len(data)/(t2-t1))
            if self.verbose: print(f'netclient: {self.port}: received {len(data)} bytes')
            self.output_queue.put(data)
        if self.verbose:
            print(f'netclient: {self.port}: disconnected')
        if s != None:
            s.close()

        try:
            self.output_queue.put(None, block=False)
        except queue.Full:
            pass
        if self.verbose: print(f"netclient: {self.port}: thread exiting")

    def statistics(self):
        self.print1stat('receive_duration', self.times_receive)
        self.print1stat('packetsize', self.sizes_receive, isInt=True)
        self.print1stat('bandwidth', self.bandwidths_receive)
        
    def print1stat(self, name, values, isInt=False):
        count = len(values)
        if count == 0:
            print('netclient: {}: port={}, count=0'.format(name, self.port))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'netclient: {}: port={}, count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'netclient: {}: port={}, count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, self.port, count, avgValue, minValue, maxValue))
    
class _NetClientMultiSource(cwipc_rawmultisource_abstract):

    def __init__(self, address : str, nTile : int, nQuality : int, verbose : bool):
        self.verbose = verbose
        self.verbose = True
        host, port = address.split(':')
        port = int(port)
        # Create a list of lists that contains all ports
        self.allPorts : List[List[int]] = []
        for tIdx in range(nTile):
            perTilePorts = []
            for qIdx in range(nQuality):
                perTilePorts.append(port)
                port += 1
            self.allPorts.append(perTilePorts)
        self.allSources : List[_NetClientSource] = []
        for tIdx in range(nTile):
            src = _NetClientSource((host, self.allPorts[tIdx][0]), verbose=verbose)
            self.allSources.append(src)

    def get_tile_count(self) -> int:
        return len(self.allSources)
    
    def get_description(self) -> cwipc_multistream_description:
        return self.allPorts

    def get_tile_source(self, tileIdx : int) -> cwipc_rawsource_abstract:
        return self.allSources[tileIdx]

    def select_tile_quality(self, tileIdx : int, qualityIdx : int) -> None:
        if self.verbose:
            print(f'netclient multisource: select tile {tileIdx} quality {qualityIdx}')
        src = self.allSources[tileIdx]
        port = self.allPorts[tileIdx][qualityIdx]
        src.switchport(port)

def cwipc_source_netclient(address : str, verbose : bool=False) -> cwipc_rawsource_abstract:
    """Return cwipc_source-like object that reads individual compressed pointclouds from a TCP-based server specified as host:port"""
    source = _NetClientSource(address, verbose=verbose)
    return source
        
def cwipc_multisource_netclient(address : str, nTile : int, nQuality : int, verbose : bool=False) -> cwipc_rawmultisource_abstract:
    """Return multisource that reads tiled streams using multiple netclients"""
    source = _NetClientMultiSource(address, nTile, nQuality, verbose=verbose)
    return source
