import sys
import os
import threading
import traceback
import queue
import struct
from typing import Optional, Dict, Any, List

from .. import cwipc_wrapper, cwipc_write, cwipc_write_debugdump, CwipcError, CWIPC_FLAGS_BINARY
from .. import codec
from ._scriptsupport import *
from ..net.abstract import *


class DropWriter(cwipc_sink_abstract):
    results = List[Dict[str, Any]]
    output_queue : queue.Queue[Optional[cwipc_wrapper]]

    def __init__(self, queuesize=5):
        self.producer = None
        self.output_queue = queue.Queue(maxsize=queuesize)
        self.count = 0
        self.results = []
        
        
    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass
        
    def set_producer(self, producer : cwipc_producer_abstract):
        self.producer = producer    
        
    def run(self):
        while (self.producer and self.producer.is_alive()) or not self.output_queue.empty():
            try:
                pc = self.output_queue.get(timeout=0.5)
                assert pc
                # xxxjack get statistics
                self.count = self.count + 1
                r : Dict[str, Any] = dict(
                    num=self.count,
                    timestamp=pc.timestamp(),
                    pointcount=pc.count(),
                )
                auxdata = pc.access_auxiliary_data()
                assert auxdata
                r["aux"] = auxdata.count()
                if auxdata != None and auxdata.count() > 0:
                    for i in range(auxdata.count()):
                        descr = auxdata.description(i)
                        r[f"_{i}"] = descr
                print(repr(r))
                pc.free()
                
            except queue.Empty:
                pass
        
        return True              
        
    def feed(self, pc : cwipc_wrapper) -> None:
        self.output_queue.put(pc)
        
    def statistics(self) -> None:
        pass        
         
def main():
    SetupStackDumper()
    parser = ArgumentParser(description="Get detailed timestamps from a capturer")
    
    args = parser.parse_args()
    beginOfRun(args)
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    source.request_auxiliary_data("timestamps")

    kwargs = {}
    writer = DropWriter()
    sourceServer = SourceServer(source, writer, args, source_name=source_name)
    sourceThread = threading.Thread(target=sourceServer.run, args=(), name="cwipc_grab.SourceServer")
    writer.set_producer(sourceThread)

    #
    # Run everything
    #
    ok = False
    try:
        sourceThread.start()

        ok = writer.run()
            
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
    del sourceServer
    endOfRun(args)
    if not ok:
        sys.exit(1)
    
if __name__ == '__main__':
    main()
    
    
    
