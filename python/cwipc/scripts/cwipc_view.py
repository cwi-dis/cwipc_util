import threading
import argparse
import traceback
from ._scriptsupport import *
from ..net.abstract import *
from ..io.visualizer import Visualizer

def main():
    SetupStackDumper()
    parser = ArgumentParser(description="View pointcloud streams", epilog="Interactive commands:\n" + Visualizer.HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nodisplay", action="store_true", help="Don't display pointclouds, only prints statistics at the end")
    parser.add_argument("--skeleton", action="store_true", help="Get and render skeleton from the capture (in stead of point cloud). Only for source --kinect or --k4aoffline")
    parser.add_argument("--rgb", action="store_true", help="Show RGB captures in addition to point clouds")
    args = parser.parse_args()
    beginOfRun(args)
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    if not args.nodisplay:
        visualizer = Visualizer(args.verbose, nodrop=args.nodrop, args=args)
    else:
        visualizer = None

    if args.skeleton:
        source.request_auxiliary_data("skeleton")
    if args.rgb:
        source.request_auxiliary_data("rgb")

    sourceServer = SourceServer(source, visualizer, args, source_name=source_name)
    sourceThread = threading.Thread(target=sourceServer.run, args=(), name="cwipc_view.SourceServer")
    if visualizer:
        visualizer.set_producer(sourceThread)
        visualizer.set_source(source)

    #
    # Run everything
    #
    try:
        sourceThread.start()

        if visualizer:
            visualizer.run()
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
    del visualizer
    del sourceServer
    endOfRun(args)
    
if __name__ == '__main__':
    main()
    
    
    
