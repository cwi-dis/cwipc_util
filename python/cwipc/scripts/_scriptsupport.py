import sys
import os
import time
import signal
import argparse
import traceback
import warnings
from typing import cast, Union, List, Callable

from .. import cwipc_wrapper, playback, cwipc_get_version, cwipc_proxy, cwipc_synthetic, cwipc_capturer
from ..net import source_netclient
from ..net import source_decoder
from ..net import source_passthrough
from ..net import source_sub
from ..net.abstract import *
from .. import filters

try:
    from .. import realsense2
except ModuleNotFoundError:
    realsense2 = None
try:
    from .. import certh
except ModuleNotFoundError:
    certh = None
try:
    from .. import kinect
except ModuleNotFoundError:
    kinect = None

if False:
    # Convoluted code warning: adding ../python directory to path so we can import subsource
    _sourcedir = os.path.dirname(__file__)
    _sourcedir = os.path.realpath(_sourcedir)
    _pardir = os.path.dirname(_sourcedir)
    _pythondir = os.path.join(_pardir, 'python')
    sys.path.append(_pythondir)

__all__ = [ 
    "SetupStackDumper",
    "ArgumentParser",
    "cwipc_genericsource_factory",
    "SourceServer",
    "beginOfRun",
    "endOfRun"
]

def _dump_app_stacks(*args) -> None:
    """Print stack traces for all threads."""
    print(f"{sys.argv[0]}: QUIT received, dumping all stacks, {len(sys._current_frames())} threads:", file=sys.stderr)
    for threadId, stack in list(sys._current_frames().items()):
        print("\nThreadID:", threadId, file=sys.stderr)
        traceback.print_stack(stack, file=sys.stderr)
        print(file=sys.stderr)


def SetupStackDumper() -> None:
    """Install signal handler so `kill --QUIT` will dump all thread stacks, for debugging."""
    if hasattr(signal, 'SIGQUIT'):
        signal.signal(signal.SIGQUIT, _dump_app_stacks)

def cwipc_genericsource_factory(args : argparse.Namespace, autoConfig : bool=False) -> tuple[cwipc_tiledsource_factory_abstract, Optional[str]]:
    """Create a cwipc_source based on command line arguments.
    Could be synthetic, realsense, kinect, proxy, certh, ...
    Returns cwipc_source object and name commonly used in cameraconfig.xml.
    """
    global realsense2
    global kinect
    name : Optional[str] = None
    source : cwipc_source_factory_abstract
    decoder_factory : Callable[[cwipc_rawsource_abstract], cwipc_source_abstract]

    if args.nodecode:
        decoder_factory = source_passthrough.cwipc_source_passthrough
    else:
        decoder_factory = source_decoder.cwipc_source_decoder
    if args.kinect:
        if kinect == None:
            print(f"{sys.argv[0]}: No support for Kinect grabber on this platform")
            sys.exit(-1)
        if autoConfig:
            source = lambda config="auto": kinect.cwipc_kinect(config) # type: ignore
        elif args.cameraconfig:
            source = lambda config=args.cameraconfig: kinect.cwipc_kinect(config) # type: ignore
        else:
            source = cast(cwipc_source_factory_abstract, kinect.cwipc_kinect)
        name = 'kinect'
    elif args.k4aoffline:
        if kinect == None or not hasattr(kinect, 'cwipc_k4aoffline'):
            print(f"{sys.argv[0]}: No support for Kinect offline grabber on this platform")
            sys.exit(-1)
        if args.cameraconfig:
            source = lambda config=args.cameraconfig: kinect.cwipc_k4aoffline(config)  # type: ignore
        else:
            source = cast(cwipc_source_factory_abstract, kinect.cwipc_k4aoffline)
        name = 'k4aoffline' # xxxjack unsure about this: do we treat kinect live and offline the same?
    elif args.realsense:
        if realsense2 == None:
            print(f"{sys.argv[0]}: No support for realsense grabber on this platform")
            sys.exit(-1)
        if autoConfig:
            source = lambda config="auto": realsense2.cwipc_realsense2(config) # type: ignore
        elif args.cameraconfig:
            source = lambda config=args.cameraconfig: realsense2.cwipc_realsense2(config) # type: ignore
        else:
            source = cast(cwipc_source_factory_abstract, realsense2.cwipc_realsense2)
        name = 'realsense'
    elif args.synthetic:
        source = lambda : cwipc_synthetic(fps=args.fps, npoints=args.npoints)
        name = None
    elif args.proxy:
        source = lambda : cwipc_proxy('', args.proxy)
        name = None
    elif args.certh:
        if certh == None:
            print(f"{sys.argv[0]}: No support for CERTH grabber on this platform")
            sys.exit(-1)
        source = lambda : certh.cwipc_certh(args.certh, args.certh_data, args.certh_metadata) # type: ignore
        name = None
    elif args.playback:
        if not os.path.isdir(args.playback):
            filename = args.playback
            playback_type = _guess_playback_type([filename])
            if not playback_type:
                print(f'{sys.argv[0]}: {filename}: unknown playback file type')
                sys.exit(-1)
            source = lambda : playback.cwipc_playback([filename], ext=playback_type, fps=args.fps, loop=args.loop, inpoint=args.inpoint, outpoint=args.outpoint, retimestamp=args.retimestamp)
            name = 'playback'
        else:
            dirname = args.playback
            playback_type = _guess_playback_type(os.listdir(dirname))
            if not playback_type:
                print(f'{sys.argv[0]}: {dirname}: should contain only one of .ply, .cwipcdump or .cwicpc files')
                sys.exit(-1)
            source = lambda : playback.cwipc_playback(dirname, ext=playback_type, fps=args.fps, loop=args.loop, inpoint=args.inpoint, outpoint=args.outpoint, retimestamp=args.retimestamp)
            name = 'playback'
    elif args.netclient:
        source = lambda : (
            decoder_factory(
                source_netclient.cwipc_source_netclient(
                    args.netclient,
                    verbose=(args.verbose > 1)
                    ),
                verbose=(args.verbose > 1)
                )
            )
        name = None
    elif args.sub:
        source = lambda : (
            decoder_factory(
                source_sub.cwipc_source_sub(
                    args.sub, 
                    verbose=(args.verbose > 1)
                    ),
                verbose=(args.verbose > 1)
                )
            )
        name = None
    else:
        # Default case: use the generic capturer.
        #
        # First we need to ensure all capturer DLLs are loaded (so they register themselves
        # with the generic capturer)
        #
        if realsense2:
            try:
                realsense2.cwipc_realsense2_dll_load()
            except RuntimeError:
                # realsense2 support could not be loaded.
                warnings.warn("realsense2 support disabled: could not load")
                kinect = None
        if kinect:
            try:
                kinect.cwipc_kinect_dll_load()
            except RuntimeError:
                # Kinect support could not be loaded.
                warnings.warn("kinect support disabled: could not load")
                kinect = None
        if autoConfig:
            source = lambda : cwipc_capturer("auto")
        elif args.cameraconfig:
            source = lambda : cwipc_capturer(args.cameraconfig)
        else:
            source = cast(cwipc_source_factory_abstract, cwipc_capturer)
        name = 'auto'
        _ = source
    return source, name

def _guess_playback_type(filenames : List[str]) -> Optional[str]:
    has_ply = False
    has_dump = False
    has_compressed = False
    for fn in filenames:
        if fn.lower().endswith('.ply'): has_ply = True
        if fn.lower().endswith('.cwipcdump'): has_dump = True
        if fn.lower().endswith('.cwicpc'): has_compressed = True
    if int(has_ply)+int(has_dump)+int(has_compressed) != 1:
        return None     # Cop-out: if we don't have exactly one file type we don't know
    if has_ply:
        return '.ply'
    if has_dump:
        return '.cwipcdump'
    if has_compressed:
        return '.cwicpc'
    return None
    
class SourceServer:
    """Wrapper class around cwipc_grabber.
    run() will send pointclouds to viewer (or other consumer) through a feed() call.
    At the end statistics can be printed."""
    
    pc_filters : List[filters.cwipc_abstract_filter]

    def __init__(self, grabber : cwipc_source_abstract, viewer, args : argparse.Namespace, source_name : Optional[str]=None):
        self.grabber = grabber
        self.verbose = args.verbose
        self.count = args.count
        self.inpoint = args.inpoint
        self.outpoint = args.outpoint
        self.cameraconfig = args.cameraconfig
        self.viewer = viewer
        self.times_grab = []
        self.pointcounts_grab = []
        self.latency_grab = []
        self.stopped = True
        self.lastGrabTime = None
        self.fps = None
        self.source_name = source_name
        if hasattr(self.grabber, 'start'):
            self.grabber.start() # type: ignore
        self.stopped = False
        self.pc_filters = []
        if args.filter:
            for fdesc in args.filter:
                filter = filters.factory(fdesc)
                self.pc_filters.append(filter)

    def __del__(self):
        self.stopped = True
        if self.grabber:
            self.grabber.free()

    def stop(self) -> None:
        if self.stopped: return
        if self.verbose: print("grab: stopping", flush=True)
        if hasattr(self.grabber, 'stop'):
            self.grabber.stop() # type: ignore
        self.stopped = True
        
    def grab_pc(self) -> Optional[cwipc_wrapper]:
        if self.lastGrabTime and self.fps:
            nextGrabTime = self.lastGrabTime + 1/self.fps
            if time.time() < nextGrabTime:
                time.sleep(nextGrabTime - time.time())
        if not self.grabber:
            return None
        if not self.grabber.available(True):
            print('grab: no pointcloud available')
            time.sleep(1)
            return None
        if not self.grabber:
            return None
        pc = self.grabber.get()
        self.lastGrabTime = time.time()
        return cast(cwipc_wrapper, pc)
        
    def run(self) -> None:
        if self.inpoint:
            if hasattr(self.grabber, 'seek'):
                result = self.grabber.seek(self.inpoint) # type: ignore
                if result:
                    print(f'grab: seek to timestamp {self.inpoint} successful', flush=True)
                else:
                    print(f'grab: Error: seek to timestamp {self.inpoint} failed', flush=True)
                    sys.exit(-1)
            else:
                print(f"grab: grabber does not support seek")
                sys.exit(-1)
                
        if self.verbose: print('grab: started', flush=True)
        while not self.stopped and not self.grabber.eof():
            t0 = time.time()
            pc = self.grab_pc()
            if not pc:
                continue
            else:
                for filter in self.pc_filters:
                    pc = filter.filter(pc)
                self.pointcounts_grab.append(pc.count())
                pc_timestamp = pc.timestamp()/1000.0
                if self.verbose: print(f'grab: captured {pc.count()} points, ts={pc.timestamp()}')
                t1 = time.time()
                if self.viewer: 
                    t = pc.timestamp()
                    if self.inpoint and t<self.inpoint:
                        continue
                    if self.outpoint and t>self.outpoint:
                        self.count = 0
                        self.stop()
                        continue
                    self.viewer.feed(pc)
                self.latency_grab.append(time.time()-pc_timestamp)
            self.times_grab.append(t1-t0)
            if self.count != None:
                self.count -= 1
                if self.count <= 0:
                    break
        if self.verbose: print('grab: stopped', flush=True)
    
    def statistics(self) -> None:
        self.print1stat('capture_duration', self.times_grab)
        self.print1stat('capture_pointcount', self.pointcounts_grab, isInt=True)
        self.print1stat('capture_latency', self.latency_grab)
        if hasattr(self.grabber, 'statistics'):
            self.grabber.statistics() # type: ignore
        for filter in self.pc_filters:
            if hasattr(filter, 'statistics'):
                filter.statistics() # type: ignore
        
    def print1stat(self, name : str, values : Union[List[int], List[float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print('grab: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'grab: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'grab: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

def ArgumentParser(*args, **kwargs) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(*args, **kwargs)
    parser.add_argument("--version", action="store_true", help="Print version and exit")
    parser.add_argument("--verbose", action="count", default=0, help="Print information about each pointcloud while it is processed. Double for even more verbosity.")
    parser.add_argument("--pausefordebug", action="store_true", help="Pause at begin and end of run (to allow attaching debugger or profiler)")
    parser.add_argument("--debuglibrary", action="append", default=[], metavar="NAME=PATH", help="Load a cwipc dynamic library from a specific path, for debugging")

    input_selection_args = parser.add_argument_group("input source selection").add_mutually_exclusive_group()
    parser.add_argument("--cameraconfig", action="store", help="Specify camera configuration file (default: ./cameraconfig.json). auto for any attached camera without configuration.")
    input_selection_args.add_argument("--realsense", action="store_true", help="Use Intel Realsense capturer (default: from camera configuration)")
    input_selection_args.add_argument("--kinect", action="store_true", help="Use Azure Kinect capturer (default: from camera configuration)")
    input_selection_args.add_argument("--k4aoffline", action="store_true", help="Use Azure Kinect pre-recorded file capturer")
    input_selection_args.add_argument("--synthetic", action="store_true", help="Use synthetic pointcloud source")
    input_selection_args.add_argument("--proxy", type=int, action="store", metavar="PORT", help="Use proxyserver pointcloud source server, proxyserver listens on PORT")
    input_selection_args.add_argument("--netclient", action="store", metavar="HOST:PORT", help="Use (compressed) pointclouds from netclient, server runs on port PORT on HOST")
    input_selection_args.add_argument("--sub", action="store", metavar="URL", help="Use DASH (compressed) pointcloud stream from URL")
    input_selection_args.add_argument("--playback", action="store", metavar="PATH", help="Use pointcloud(s) from ply or cwipcdump file or directory (in alphabetical order)")
    input_selection_args.add_argument("--certh", action="store", metavar="URL", help="Use Certh pointcloud stream from Rabbitmq server URL")

    input_args = parser.add_argument_group("input arguments")
    input_args.add_argument("--nodecode", action="store_true", help="Receive uncompressed pointclouds with --netclient and --sub (default: compressed with cwipc_codec)")
    input_args.add_argument("--certh_data", action="store", metavar="NAME", help="Use NAME for certh data exchange (default: VolumetricData)", default="VolumetricData")
    input_args.add_argument("--certh_metadata", action="store", metavar="NAME", help="Use NAME for certh metadata exchange (default: VolumetricMetaData)", default="VolumetricMetaData")
    input_args.add_argument("--loop", action="store_true", help="With --playback loop the contents in stead of terminating after the last file")
    input_args.add_argument("--npoints", action="store", metavar="N", type=int, help="Limit number of points (approximately) in synthetic pointcoud", default=0)
    input_args.add_argument("--fps", action="store", type=int, help="Limit playback rate to FPS (for some capturers)", default=0)
    input_args.add_argument("--retimestamp", action="store_true", help="Set timestamps to wall clock in stead of recorded timestamps (for some grabbers)")
    input_args.add_argument("--count", type=int, action="store", metavar="N", help="Stop after receiving N pointclouds")
    input_args.add_argument("--inpoint", type=int, action="store", metavar="N", help="Start at frame with timestamp > N")
    input_args.add_argument("--outpoint", type=int, action="store", metavar="N", help="Stop at frame with timestamp >= N")
    input_args.add_argument("--nodrop", action="store_true", help="Attempt to store all captures by not dropping frames. Only works for some prerecorded capturers.")
    input_args.add_argument("--filter", action="append", metavar="FILTERDESC", help="After capture apply a filter to each point cloud. Multiple filters are applied in order.")
    input_args.add_argument("--help_filters", action="store_true", help="List available filters and exit")
    return parser
            
def beginOfRun(args : argparse.Namespace) -> None:
    """Optionally pause execution"""
    if args.version:
        print(cwipc_get_version())
        sys.exit(0)
    if args.help_filters:
        filters.help()
        sys.exit(0)
    if args.pausefordebug:
        answer=None
        while answer != 'Y':
            print(f"{sys.argv[0]}: starting, pid={os.getpid()}. Press Y to continue -", flush=True)
            answer = sys.stdin.readline()
            answer = answer.strip()
        print(f"{sys.argv[0]}: started.")
    for debuglibrary in args.debuglibrary:
        try:
            name, path = debuglibrary.split('=')
        except ValueError:
            name = path = None
        print(f"{sys.argv[0]}: load {name} from {path}", file=sys.stderr)
        if name == 'cwipc_util':
            from ..util import cwipc_util_dll_load
            cwipc_util_dll_load(path)
        elif name == 'cwipc_codec':
            from _cwipc_codec import cwipc_codec_dll_load
            cwipc_codec_dll_load(path)
        elif name == 'cwipc_realsense2':
            from _cwipc_realsense2 import cwipc_realsense2_dll_load
            cwipc_realsense2_dll_load(path)
        elif name == 'cwipc_kinect':
            from _cwipc_kinect import cwipc_kinect_dll_load
            cwipc_kinect_dll_load(path)
        elif name == 'signals-unity-bridge':
            from ..net.source_sub import _signals_unity_bridge_dll
            _signals_unity_bridge_dll(path)
        elif name == 'bin2dash':
            from ..net.sink_bin2dash import _bin2dash_dll
            _bin2dash_dll(path)
        else:
            print(f"{sys.argv[0]}: incorrect --debuglibrary argument: {args.debuglibrary}")
            print(f"{sys.argv[0]}: allowed values: cwipc_util, cwipc_codec, cwipc_realsense2, cwipc_kinect, signals-unity-bridge, bin2dash")
            sys.exit(1)

def endOfRun(args : argparse.Namespace) -> None:
    """Optionally pause execution"""
    if args.pausefordebug:
        answer=None
        while answer != 'Y':
            print(f"{sys.argv[0]}: stopping, pid={os.getpid()}. Press Y to continue -", flush=True)
            answer = sys.stdin.readline()
            answer = answer.strip()
        print(f"{sys.argv[0]}: stopped.")
