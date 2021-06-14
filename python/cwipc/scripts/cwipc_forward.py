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
import cwipc.net.sink_passthrough
import cwipc.net.sink_bin2dash
from ._scriptsupport import *

def main():
    SetupStackDumper()
    parser = ArgumentParser(description="Forward pointcloud streams", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--noforward", action="store_true", help="Don't forward pointclouds, only prints statistics at the end")
    
    output_selection_args = parser.add_argument_group("output selection").add_mutually_exclusive_group()
    output_selection_args.add_argument("--port", action="store", default=4303, type=int, metavar="PORT", help="Port to serve compressed pointclouds on (default: 4303)")
    output_selection_args.add_argument("--bin2dash", action="store", metavar="URL", help="Send compressed data to bin2dash URL in stead of serving. Example URL:  https://vrt-evanescent.viaccess-orca.com/pctest/")
    
    output_args = parser.add_argument_group("output arguments")
    output_args.add_argument("--seg_dur", action="store", type=int, metavar="MS", help="Bin2dash segment duration (milliseconds, default 10000)")
    output_args.add_argument("--timeshift_buffer", action="store", type=int, metavar="MS", help="Bin2dash timeshift buffer depth (milliseconds, default 30000)")
    output_args.add_argument("--noencode", action="store_true", help="Send uncompressed pointclouds (default: use cwipc_codec encoder)")
#    parser.add_argument("--octree_bits", action="store", type=int, metavar="N", help="Override encoder parameter (depth of octree)")
#    parser.add_argument("--jpeg_quality", action="store", type=int, metavar="N", help="Override encoder parameter (jpeg quality)")
    args = parser.parse_args()
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
#    encparams = cwipc.codec.cwipc_encoder_params(False, 1, 1.0, 9, 85, 16, 0, 0)
#    if args.octree_bits or args.jpeg_quality:
#        if args.octree_bits:
#            encparams.octree_bits = args.octree_bits
#        if args.jpeg_quality:
#            encparams.jpeg_quality = args.jpeg_quality
    if args.noencode:
        encoder_factory = cwipc.net.sink_passthrough.cwipc_sink_passthrough
    else:
        encoder_factory = cwipc.net.sink_encoder.cwipc_sink_encoder
    if args.noforward:
        forwarder = None
    elif args.bin2dash:
        forwarder = encoder_factory(
            cwipc.net.sink_bin2dash.cwipc_sink_bin2dash(
                args.bin2dash,
                seg_dur_in_ms=args.seg_dur,
                timeshift_buffer_depth_in_ms=args.timeshift_buffer, 
                verbose=(args.verbose > 1),
                nodrop=args.nodrop
            ),
            verbose=(args.verbose > 1),
            nodrop=args.nodrop
        )
    else:
        forwarder = encoder_factory(
            cwipc.net.sink_netserver.cwipc_sink_netserver(
                args.port, 
                verbose=(args.verbose > 1),
                nodrop=args.nodrop
            ),
            verbose=(args.verbose > 1),
            nodrop=args.nodrop
        )

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
    
    
    
