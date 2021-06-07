import sys
import os
import time
import threading
import time
import argparse
import traceback
import cwipc
import cwipc.net.sink_netserver
import cwipc.net.sink_encoder
from ._scriptsupport import *

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
    encoder_factory = cwipc.net.sink_encoder.cwipc_sink_encoder
    if not args.noforward:
        forwarder = encoder_factory(
            cwipc.net.sink_netserver.cwipc_sink_netserver(
                args.port, 
                verbose=(args.verbose > 1),
                nodrop=args.nodrop
            ),
            verbose=(args.verbose > 1),
            nodrop=args.nodrop
        )
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

        if forwarder and hasattr(forwarder, 'start'):
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
    if forwarder and hasattr(forwarder, 'stop'):
        forwarder.stop()
    if forwarder and hasattr(forwarder, 'statistics'):
        forwarder.statistics()
    sourceServer.statistics()
    
if __name__ == '__main__':
    main()
    
    
    
