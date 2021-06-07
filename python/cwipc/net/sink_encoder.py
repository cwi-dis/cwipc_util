import os
import threading
import socket
import select
import time
import queue
import cwipc

try:
    import cwipc.codec
except ModuleNotFoundError:
    cwipc.codec = None

class _Sink_Encoder(threading.Thread):
    
    SELECT_TIMEOUT=0.1
    QUEUE_FULL_TIMEOUT=0.001
    
    def __init__(self, sink, verbose=False, nodrop=False):
        threading.Thread.__init__(self)
        self.sink = sink
        self.producer = None
        self.nodrop = nodrop
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.nodrop = nodrop
        self.stopped = False
        self.started = False
        self.times_encode = []
        self.pointcounts = []
         
    def start(self):
        threading.Thread.start(self)
        self.sink.start()
        self.started = True
        
    def stop(self):
        if self.verbose: print(f"encoder: stopping thread")
        self.stopped = True
        self.sink.stop()
        if self.started:
            self.join()
        
    def set_producer(self, producer):
        self.producer = producer
        self.sink.set_producer(producer)
        
    def is_alive(self):
        return not self.stopped  
        
    def run(self):
        if self.verbose: print(f"encoder: thread started")
        try:
            while not self.stopped and self.producer and self.producer.is_alive():
                pc = self.queue.get()
                if not pc:
                    print(f"encoder: get() returned None")
                    continue
                self.pointcounts.append(pc.count())
                t1 = time.time()
                cpc = self._encode_pc(pc)
                t2 = time.time()
                self.sink.feed(cpc)
                pc.free()
                self.times_encode.append(t2-t1)
        finally:
            self.stopped = True
            if self.verbose: print(f"encoder: thread stopping")
        
    def feed(self, pc):
        try:
            if self.nodrop:
                self.queue.put(pc)
            else:
                self.queue.put(pc, timeout=self.QUEUE_FULL_TIMEOUT)
        except queue.Full:
            if self.verbose: print(f"encoder: queue full, drop pointcloud")
            pc.free()
    
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
        self.print1stat('encode_duration', self.times_encode)
        self.print1stat('pointcount', self.pointcounts)
        self.sink.statistics()
        
    def print1stat(self, name, values, isInt=False):
        count = len(values)
        if count == 0:
            print('encoder: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'encoder: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'encoder: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

def cwipc_sink_encoder(sink, verbose=False, nodrop=False):
    """Create a cwipc_sink object that serves compressed pointclouds on a TCP network port"""
    if cwipc.codec == None:
        raise RuntimeError("cwipc_sink_encoder: requires cwipc.codec with is not available")
    return _Sink_Encoder(sink, verbose=verbose, nodrop=nodrop)