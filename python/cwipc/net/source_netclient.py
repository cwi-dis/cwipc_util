import time
import os
import socket
import threading
import queue
import cwipc
import struct

class _NetClientSource(threading.Thread):
    
    QUEUE_WAIT_TIMEOUT=1
    
    
    def __init__(self, address, verbose=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._NetClientSource'
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
        self.verbose = verbose
        self.queue = queue.Queue()
        self.times_receive = []
        self.sizes_receive = []
        self.bandwidths_receive = []
        
    def free(self):
        pass
        
    def start(self):
        assert not self.running
        if self.verbose: print('netclient: start')
        self.running = True
        threading.Thread.start(self)
        
    def stop(self):
        if self.verbose: print('netclient: stop')
        self.running = False
        self.queue.put(None)
        self.join()
        
    def eof(self):
        return self.queue.empty() and self._conn_refused
    
    def available(self, wait=False):
        if not self.queue.empty():
            return True
        if not wait or self._conn_refused:
            return False
        # Note: the following code may reorder packets...
        try:
            packet = self.queue.get(timeout=self.QUEUE_WAIT_TIMEOUT)
            if packet:
                self.queue.put(packet)
            return not not packet
        except queue.Empty:
            return False
        
    def get(self):
        if self.eof():
            return None
        packet = self.queue.get()
        return packet

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
                t1 = time.time()
                packet = b''
                while True:
                    data = s.recv(8192)
                    if not data: break
                    packet += data
                hdr = packet[:16]
                packet = packet[16:]
                h_fourcc, h_length, h_timestamp = struct.unpack("=4sLQ", hdr)
                assert h_fourcc.decode("ascii") == "0iwc"
                assert h_length == len(packet)
                # Ignore h_timestamp for now.
                t2 = time.time()
                if t2 == t1: t2 = t1 + 0.0005
                self.times_receive.append(t2-t1)
                self.sizes_receive.append(len(packet))
                self.bandwidths_receive.append(len(packet)/(t2-t1))
                if self.verbose: print(f'netclient: received {len(packet)} bytes')
                self.queue.put(packet)
        self.queue.put(None)
        if self.verbose: print(f"netclient: thread exiting")

    def statistics(self):
        self.print1stat('receive_duration', self.times_receive)
        self.print1stat('packetsize', self.sizes_receive, isInt=True)
        self.print1stat('bandwidth', self.bandwidths_receive)
        
    def print1stat(self, name, values, isInt=False):
        count = len(values)
        if count == 0:
            print('netclient: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'netclient: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'netclient: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))
    
def cwipc_source_netclient(address, verbose=False):
    """Return cwipc_source-like object that reads individual compressed pointclouds from a TCP-based server specified as host:port"""
    source = _NetClientSource(address, verbose=verbose)
    return source
        
