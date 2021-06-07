import sys
import os
import time
import threading
import socket
import select
import time
import argparse
import traceback
import queue
import cwipc
try:
    import cwipc.codec
except ModuleNotFoundError:
    cwipc.codec = None
from ._scriptsupport import *

class Forwarder(threading.Thread):
    
    SELECT_TIMEOUT=0.1
    QUEUE_FULL_TIMEOUT=0.001
    
    def __init__(self, port, verbose=False, nodrop=False):
        threading.Thread.__init__(self)
        self.producer = None
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.nodrop = nodrop
        self.stopped = False
        self.times_forward = []
        self.sizes_forward = []
        self.bandwidths_forward = []
        self.socket = socket.socket()
        self.socket.bind(('', port))
        self.socket.listen()
         
    def set_producer(self, producer):
        self.producer = producer
        
    def is_alive(self):
        return not self.stopped  
        
    def run(self):
        while self.producer and self.producer.is_alive():
            readable, _, _ = select.select([self.socket], [], [], self.SELECT_TIMEOUT)
            if self.socket in readable:
                t1 = time.time()
                connSocket, other = self.socket.accept()
                if self.verbose:
                    print(f"forwarder: accepted connection from {other}")
                data = self.queue.get()
                connSocket.sendall(data)
                connSocket.close()
                t2 = time.time()
                connSocket = None
                self.times_forward.append(t2-t1)
                datasize = len(data)
                self.sizes_forward.append(datasize)
                self.bandwidths_forward.append(datasize/(t2-t1))
        self.stopped = True
        
    def feed(self, pc):
        cpc = self._encode_pc(pc)
        try:
            if self.nodrop:
                self.queue.put(cpc)
            else:
                self.queue.put(cpc, timeout=self.QUEUE_FULL_TIMEOUT)
        except queue.Full:
            pass
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
        self.print1stat('connection_duration', self.times_forward)
        self.print1stat('datasize', self.sizes_forward)
        self.print1stat('bandwidth', self.bandwidths_forward)
        
    def print1stat(self, name, values, isInt=False):
        count = len(values)
        if count == 0:
            print('forwarder: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'forwarder: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'forwarder: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))
    
def main():
    SetupStackDumper()
    parser = ArgumentParser(description="Forward pointcloud streams", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--noforward", action="store_true", help="Don't forward pointclouds, only prints statistics at the end")
    parser.add_argument("--port", action="store", default=4303, type=int, metavar="PORT", help="Port to serve compressed pointclouds on (default: 4303)")
    args = parser.parse_args()
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    if not args.noforward:
        forwarder = Forwarder(args.port, verbose=args.verbose)
    else:
        forwarder = None

    sourceServer = SourceServer(source, forwarder, count=args.count, inpoint=args.inpoint, outpoint=args.outpoint, verbose=args.verbose, source_name=source_name)
    sourceThread = threading.Thread(target=sourceServer.run, args=())
    if forwarder:
        forwarder.set_producer(sourceThread)

    #
    # Run everything
    #
    try:
        sourceThread.start()

        if forwarder:
            forwarder.start()
            
        sourceThread.join()
    except KeyboardInterrupt:
        print("Interrupted.")
        sourceServer.stop()
    except:
        traceback.print_exc()
    
    #
    # It is safe to call join (or stop) multiple times, so we ensure to cleanup
    #
    sourceServer.stop()
    sourceThread.join()
    if forwarder:
        forwarder.join()
        forwarder.statistics()
    sourceServer.statistics()
    
if __name__ == '__main__':
    main()
    
    
    
