import sys
import argparse
import threading
import traceback
import queue
import csv
from typing import Optional, Dict, Any, List, Iterable

from .. import cwipc_wrapper, cwipc_write, cwipc_write_debugdump, CwipcError, CWIPC_FLAGS_BINARY
from .. import codec
from ._scriptsupport import *
from ..net.abstract import *


class DropWriter(cwipc_sink_abstract):
    results : List[Dict[str, Any]]
    output_queue : queue.Queue[Optional[cwipc_wrapper]]
    csvwriter : Optional[csv.DictWriter]
    csvkeys : List[str]
    details : bool

    def __init__(self, args : argparse.Namespace, queuesize=5):
        self.producer = None
        self.output_queue = queue.Queue(maxsize=queuesize)
        self.count = 0
        self.details = args.details
        self.results = []
        self.csvwriter = None
        self.csvkeys = []
        self.previous_timestamp = None
        
        
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
                pc_timestamp = pc.timestamp()
                r : Dict[str, Any] = dict(
                    num=self.count,
                    timestamp=pc_timestamp,
                    pointcount=pc.count(),
                )
                if self.previous_timestamp != None:
                    r["frame_duration"] = pc_timestamp - self.previous_timestamp
                self.previous_timestamp = pc_timestamp
                auxdata = pc.access_auxiliary_data()
                assert auxdata
                r["aux"] = auxdata.count()
                if auxdata != None and auxdata.count() > 0:
                    for i in range(auxdata.count()):
                        auxname = auxdata.name(i)
                        descr_dict = auxdata._parse_aux_description(auxdata.description(i))
                        for k, v in descr_dict.items():
                            r[f"{auxname}.{k}"] = v
                        delta_time_depth = pc_timestamp - descr_dict["depth_timestamp"]
                        delta_time_color = pc_timestamp - descr_dict["color_timestamp"]
                        r[f"{auxname}.color_age"] = delta_time_color
                        r[f"{auxname}.depth_age"] = delta_time_depth
                    self.writerecord(r)
                pc.free()
                
            except queue.Empty:
                pass
        
        return True              
        
    def feed(self, pc : cwipc_wrapper) -> None:
        self.output_queue.put(pc)
        
    def writerecord(self, record : Dict[str, Any]) -> None:
        if self.csvwriter == None:
            self.init_csv(record)
        assert self.csvwriter
        self.csvwriter.writerow(self.filter_record(record))
        sys.stdout.flush()

    def init_csv(self, record : Dict[str, Any]) -> None:
        fieldnames = record.keys()
        self.csvkeys = self.filter_keys(fieldnames)
        self.csvwriter = csv.DictWriter(sys.stdout, self.csvkeys)
        self.csvwriter.writeheader()

    def filter_record(self, record: Dict[str, Any]) -> Dict[str, any]:
        rv = {}
        for k, v in record.items():
            if k in self.csvkeys:
                rv[k] = v
        return rv
    
    def filter_keys(self, keys : Iterable[str]) -> List[str]:
        if self.details:
            return list(keys)
        rv = []
        for k in keys:
            if k in {"num", "timestamp", "pointcount", "frame_duration"}:
                rv.append(k)
            elif "age" in k:
                rv.append(k)
        return rv
    
    def statistics(self) -> None:
        pass        
         
def main():
    SetupStackDumper()
    
    parser = ArgumentParser(description="Get detailed timestamps from a capturer")
    parser.add_argument("--details", action="store_true", help="Output detailed timing records")

    args = parser.parse_args()
    
    beginOfRun(args)
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    source.request_auxiliary_data("timestamps")

    kwargs = {}
    writer = DropWriter(args=args)
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
    
    
    
