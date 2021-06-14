import sys
import os
import time
import threading
import traceback
import queue
import socket
import struct
from .. import CWIPC_POINT_PACKETHEADER_MAGIC
from ._scriptsupport import *

class Sender:
    
    def __init__(self, host, port, verbose=False):
        self.producer = None
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.socket = socket.socket()
        self.socket.connect((host, port))
        
    def set_producer(self, producer):
        self.producer = producer    
        
    def run(self):
        while self.producer and self.producer.is_alive():
            try:
                pc = self.queue.get(timeout=0.033)
                ok = self.send_pc(pc)
                pc.free()
            except queue.Empty:
                pass
        
    def feed(self, pc):
        try:
            self.queue.put(pc, timeout=0.5)
        except queue.Full:
            pc.free()
            
    def send_pc(self, pc):
        data = pc.get_bytes()
        cellsize = pc.cellsize()
        timestamp = pc.timestamp()
        header = struct.pack("<iiqfi", CWIPC_POINT_PACKETHEADER_MAGIC, len(data), timestamp, cellsize, 0)
        x = self.socket.send(header)
        y = self.socket.send(data)

 

def main():
    SetupStackDumper()
    parser = ArgumentParser(description="Send pointcloud stream to cwipc_proxy")
    parser.add_argument("host", action="store", help="Hostname where cwipc_proxy server is running")
    parser.add_argument("port", type=int, action="store", help="Port where cwipc_proxy server is running")
    args = parser.parse_args()

    sourceFactory, _ = cwipc_genericsource_factory(args)
    source = sourceFactory()
    sender = Sender(args.host, args.port)
    sourceServer = SourceServer(source, sender, count=args.count, verbose=args.verbose)
    sourceThread = threading.Thread(target=sourceServer.run, args=())
    if sender:
        sender.set_producer(sourceThread)

    #
    # Run everything
    #
    try:
        sourceThread.start()

        if sender:
            sender.run()
            sourceServer.stop()
            
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
    sourceServer.statistics()
    
if __name__ == '__main__':
    main()
    
    
    
