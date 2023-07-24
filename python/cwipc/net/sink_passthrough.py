import os
import threading
import socket
import select
import time
import queue
import cwipc
from typing import Optional, List, Any
from .abstract import VRT_4CC, vrt_fourcc_type, cwipc_producer_abstract, cwipc_rawsink_abstract, cwipc_sink_abstract

class _Sink_Passthrough(threading.Thread, cwipc_sink_abstract):
    """A sink object that serializes pointclouds and forwards them to a rawsink."""
    
    FOURCC="cwi0"
    SELECT_TIMEOUT=0.1
    QUEUE_FULL_TIMEOUT=0.001

    def __init__(self, sink, verbose=False, nodrop=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._SinkPassthrough'
        self.sink = sink
        if hasattr(self.sink, 'set_fourcc'):
            self.sink.set_fourcc(self.FOURCC)
        self.producer = None
        self.nodrop = nodrop
        self.input_queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.nodrop = nodrop
        self.stopped = False
        self.started = False
        self.pointcounts = []
         
    def set_encoder_params(self, **kwargs):
        """Specify the parameters for the encoder."""
        raise RuntimeError("cwipc_sink_passthrough: no encoder parameters supported")
        
    def start(self):
        threading.Thread.start(self)
        self.sink.start()
        self.started = True
        
    def stop(self):
        if self.verbose: print(f"passthrough: stopping thread")
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
        if self.verbose: print(f"passthrough: thread started")
        try:
            while not self.stopped and self.producer and self.producer.is_alive():
                pc = self.input_queue.get()
                if not pc:
                    print(f"passthrough: get() returned None")
                    continue
                self.pointcounts.append(pc.count())
                cpc = pc.get_packet()
                self.sink.feed(cpc)
                pc.free()
        finally:
            self.stopped = True
            if self.verbose: print(f"passthrough: thread stopping")
        
    def feed(self, pc):
        try:
            if self.nodrop:
                self.input_queue.put(pc)
            else:
                self.input_queue.put(pc, timeout=self.QUEUE_FULL_TIMEOUT)
        except queue.Full:
            if self.verbose: print(f"passthrough: queue full, drop pointcloud")
            pc.free()

    def statistics(self):
        self.print1stat('pointcount', self.pointcounts)
        if hasattr(self.sink, 'statistics'):
            self.sink.statistics()
        
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

def cwipc_sink_passthrough(sink : cwipc_rawsink_abstract, verbose=False, nodrop=False) -> cwipc_sink_abstract:
    """Create a cwipc_sink object sends serialized uncompressed pointclouds to another sink"""
    return _Sink_Passthrough(sink, verbose=verbose, nodrop=nodrop)