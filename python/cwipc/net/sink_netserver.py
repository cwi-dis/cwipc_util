import os
import threading
import socket
import select
import time
import queue
import cwipc
import struct

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

class _Sink_NetServer(threading.Thread):
    
    SELECT_TIMEOUT=0.1
    QUEUE_FULL_TIMEOUT=0.001
    
    def __init__(self, port, verbose=False, nodrop=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._Sink_NetServer'
        self.producer = None
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.nodrop = nodrop
        self.stopped = False
        self.started = False
        self.fourcc = None
        self.times_forward = []
        self.sizes_forward = []
        self.bandwidths_forward = []
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', port))
        self.socket.listen()
         
    def start(self):
        threading.Thread.start(self)
        self.started = True
        
    def stop(self):
        if self.verbose: print(f"netserver: stopping thread")
        self.stopped = True
        self.socket.close()
        if self.started:
            self.join()
        
    def set_fourcc(self, fourcc):
        self.fourcc = fourcc

    def set_producer(self, producer):
        self.producer = producer
        
    def is_alive(self):
        return not self.stopped  
        
    def run(self):
        if self.verbose: print(f"netserver: thread started")
        try:
            while not self.stopped and self.producer and self.producer.is_alive():
                readable, _, errorable = select.select([self.socket], [], [], self.SELECT_TIMEOUT)
                if self.socket in errorable:
                    continue
                if self.socket in readable:
                    t1 = time.time()
                    connSocket, other = self.socket.accept()
                    if self.verbose:
                        print(f"netserver: accepted connection from {other}")
                    data = self.queue.get()
                    hdr = self._gen_header(data)
                    connSocket.sendall(hdr + data)
                    connSocket.close()
                    t2 = time.time()
                    if t2 == t1: t2 = t1 + 0.0005
                    connSocket = None
                    self.times_forward.append(t2-t1)
                    datasize = len(data)
                    self.sizes_forward.append(datasize)
                    self.bandwidths_forward.append(datasize/(t2-t1))
        finally:
            self.stopped = True
            if self.verbose: print(f"netserver: thread stopping")
    
    def _gen_header(self, data):
        datalen = len(data)
        timestamp = int(time.time() * 1000)
        return struct.pack("=LLQ", VRT_4CC(self.fourcc), datalen, timestamp)
    
    def feed(self, data):
        try:
            if self.nodrop:
                self.queue.put(data)
            else:
                self.queue.put(data, timeout=self.QUEUE_FULL_TIMEOUT)
        except queue.Full:
            if self.verbose: print(f"netserver: queue full, drop packet")            
    
    def _encode_pc(self, pc):
        encparams = cwipc.codec.cwipc_encoder_params(False, 1, 1.0, 9, 85, 16, 0, 0)
        enc = cwipc.codec.cwipc_new_encoder(params=encparams)
        enc.feed(pc)
        gotData = enc.available(True)
        assert gotData
        data = enc.get_bytes()
        pc.free()
        enc.free()
        return data

    def statistics(self):
        self.print1stat('connection_duration', self.times_forward)
        self.print1stat('packetsize', self.sizes_forward)
        self.print1stat('bandwidth', self.bandwidths_forward)
        
    def print1stat(self, name, values, isInt=False):
        count = len(values)
        if count == 0:
            print('netserver: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'netserver: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'netserver: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

def cwipc_sink_netserver(port, verbose=False, nodrop=False):
    """Create a cwipc_sink object that serves compressed pointclouds on a TCP network port"""
    return _Sink_NetServer(port, verbose=verbose, nodrop=nodrop)
