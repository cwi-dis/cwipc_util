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
    
    FOURCC="cwi1"
    SELECT_TIMEOUT=0.1
    QUEUE_FULL_TIMEOUT=0.001
    
    def __init__(self, sink, verbose=False, nodrop=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._Sink_Encoder'
        self.sink = sink
        if hasattr(self.sink, 'set_fourcc'):
            self.sink.set_fourcc(self.FOURCC)
        self.producer = None
        self.nodrop = nodrop
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.nodrop = nodrop
        self.stopped = False
        self.started = False
        self.times_encode = []
        self.pointcounts = []
        self.encoder_group = None
        self.encoders = []
        
        self.tiledescriptions = [{}]
        self.octree_bits = None
        self.jpeg_quality = None
        
    def set_encoder_params(self, tiles=False, octree_bits=None, jpeg_quality=None):
        if tiles == None: tiles = [{}]
        self.tiledescriptions = tiles
        self.octree_bits = octree_bits
        self.jpeg_quality = jpeg_quality
        
    def start(self):
        self._init_encoders()
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
                self.encoder_group.feed(pc)
                packets = []
                for i in range(len(self.encoders)):
                    got_data = self.encoders[i].available(True)
                    assert got_data
                    cpc = self.encoders[i].get_bytes()
                    packets.append(cpc)
                t2 = time.time()
                
                if len(packets) == 1:
                    self.sink.feed(packets[0])
                else:
                    for i in range(len(packets)):
                        self.sink.feed(packets[i], stream_index=i)
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
    
    def _init_encoders(self):
        if not self.octree_bits:
            self.octree_bits = 9
        if type(self.octree_bits) != type([]):
            self.octree_bits = [self.octree_bits]
       
        if not self.jpeg_quality:
            self.jpeg_quality = 85
        if type(self.jpeg_quality) != type([]):
            self.jpeg_quality = [self.jpeg_quality]
            
        voxelsize = 0
        
        if self.verbose:
            print(f'encoder: creating {len(self.tiledescriptions)*len(self.octree_bits)*len(self.jpeg_quality)} encoders/streams')
        
        self.encoder_group = cwipc.codec.cwipc_new_encodergroup()
        for tile in range(len(self.tiledescriptions)):
            for octree_bits in self.octree_bits:
                for jpeg_quality in self.jpeg_quality:
                    srctile = self.tiledescriptions[tile].get('ncamera', tile)
                    encparams = cwipc.codec.cwipc_encoder_params(False, 1, 1.0, octree_bits, jpeg_quality, 16, srctile, voxelsize)
                    encoder = self.encoder_group.addencoder(params=encparams)
                    self.encoders.append(encoder)
                    if hasattr(self.sink, 'add_streamDesc'):
                        # Our sink can handle multiple tiles/quality streams.
                        # Initialize to the best of our knowledge
                        if not 'normal' in self.tiledescriptions[tile]:
                            print(f'encoder: warning: tile {tile} description has no normal vector: {self.tiledescriptions[tile]}')
                        normal = self.tiledescriptions[tile].get("normal", dict(x=0, y=0, z=0))
                        streamNum = self.sink.add_streamDesc(tile, normal['x'], normal['y'], normal['z'])
                        if self.verbose:
                            print(f'encoder: streamNum={streamNum}, tile={tile}, srctile={srctile}, normal={normal}, octree_bits={octree_bits}, jpeg_quality={jpeg_quality}')
                    else:
                        # Single stream sink.
                        streamNum = 0
                    assert streamNum == len(self.encoders)-1 # Fails if multi-stream not supported by network sink.

    def statistics(self):
        self.print1stat('encode_duration', self.times_encode)
        self.print1stat('pointcount', self.pointcounts)
        if hasattr(self.sink, 'statistics'):
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