import sys
import os
import time
import signal
import argparse
import traceback

import cwipc
import cwipc.playback

try:
    import cwipc.realsense2
except ModuleNotFoundError:
    cwipc.realsense2 = None
try:
    import cwipc.certh
except ModuleNotFoundError:
    cwipc.certh = None
try:
    import cwipc.kinect
except ModuleNotFoundError:
    cwipc.kinect = None

if False:
    # Convoluted code warning: adding ../python directory to path so we can import subsource
    _sourcedir = os.path.dirname(__file__)
    _sourcedir = os.path.realpath(_sourcedir)
    _pardir = os.path.dirname(_sourcedir)
    _pythondir = os.path.join(_pardir, 'python')
    sys.path.append(_pythondir)

__all__ = [ 
    "SetupStackDumper",
    "GrabberArgumentParser",
    "ArgumentParser",
    "cwipc_genericsource_factory",
    "SourceServer",
]

def _dump_app_stacks(*args):
    """Print stack traces for all threads."""
    print(f"{sys.argv[0]}: QUIT received, dumping all stacks, {len(sys._current_frames())} threads:", file=sys.stderr)
    for threadId, stack in list(sys._current_frames().items()):
        print("\nThreadID:", threadId, file=sys.stderr)
        traceback.print_stack(stack, file=sys.stderr)
        print(file=sys.stderr)


def SetupStackDumper():
    """Install signal handler so `kill --QUIT` will dump all thread stacks, for debugging."""
    if hasattr(signal, 'SIGQUIT'):
        signal.signal(signal.SIGQUIT, _dump_app_stacks)

def cwipc_genericsource_factory(args):
    """Create a cwipc_source based on command line arguments.
    Could be synthetic, realsense, kinect, proxy, certh, ...
    Returns cwipc_source object and name commonly used in cameraconfig.xml.
    """
    name = None
    if args.kinect:
        if cwipc.kinect == None:
            print(f"{sys.argv[0]}: No support for Kinect grabber on this platform")
            sys.exit(-1)
        source = cwipc.kinect.cwipc_kinect
        name = 'kinect'
    elif args.k4aoffline:
        if cwipc.kinect == None or not hasattr(cwipc.kinect, 'cwipc_k4aoffline'):
            print(f"{sys.argv[0]}: No support for Kinect offline grabber on this platform")
            sys.exit(-1)
        source = cwipc.kinect.cwipc_k4aoffline
        name = 'kinect' # xxxjack unsure about this: do we treat kinect live and offline the same?
    
    elif args.synthetic:
        source = lambda : cwipc.cwipc_synthetic(fps=args.fps, npoints=args.npoints)
        name = None
    elif args.proxy:
        source = lambda : cwipc.cwipc_proxy('', args.proxy)
        name = None
    elif args.certh:
        if cwipc.certh == None:
            print(f"{sys.argv[0]}: No support for CERTH grabber on this platform")
            sys.exit(-1)
        source = lambda : cwipc.certh.cwipc_certh(args.certh, args.certh_data, args.certh_metadata)
        name = None
    elif args.file:
        source = lambda : cwipc.playback.cwipc_playback([args.file], ply=not args.dump, fps=args.fps, loop=True)
        name = None
    elif args.dir:
        source = lambda : cwipc.playback.cwipc_playback(args.dir, ply=not args.dump, fps=args.fps, loop=True)
        name = None
    else:
        if cwipc.realsense2 == None:
            print(f"{sys.argv[0]}: No support for realsense grabber on this platform")
            sys.exit(-1)
        source = cwipc.realsense2.cwipc_realsense2
        name = 'realsense'
    return source, name

class SourceServer:
    """Wrapper class around cwipc_grabber.
    run() will send pointclouds to viewer (or other consumer) through a feed() call.
    At the end statistics can be printed."""
    
    def __init__(self, grabber, viewer=None, count=None, verbose=False):
        self.verbose = verbose
        self.grabber = grabber
        self.viewer = viewer
        self.count = count
        self.times_grab = []
        self.pointcounts_grab = []
        self.stopped = False
        self.lastGrabTime = None
        self.fps = None
        
    def __del__(self):
        self.stopped = True
        if self.grabber:
            self.grabber.free()

    def stop(self):
        if self.stopped: return
        if self.verbose: print("grab: stopping", flush=True)
        self.stopped = True
        
    def grab_pc(self):
        if self.lastGrabTime and self.fps:
            nextGrabTime = self.lastGrabTime + 1/self.fps
            if time.time() < nextGrabTime:
                time.sleep(nextGrabTime - time.time())
        if not self.grabber.available(True):
                print('grab: no pointcloud available')
                time.sleep(1)
                return None
        pc = self.grabber.get()
        self.lastGrabTime = time.time()
        return pc
        
    def run(self):
        if self.verbose: print('grab: started', flush=True)
        while not self.stopped and not self.grabber.eof():
            t0 = time.time()
            pc = self.grab_pc()
            if not pc:
                continue
            else:
                self.pointcounts_grab.append(pc.count())
                if self.verbose: print(f'grab: captured {pc.count()} points')
                t1 = time.time()
                if self.viewer: self.viewer.feed(pc)
            self.times_grab.append(t1-t0)
            if self.count != None:
                self.count -= 1
                if self.count <= 0:
                    break
        if self.verbose: print('grab: stopped', flush=True)
            
    def statistics(self):
        self.print1stat('capture_duration', self.times_grab)
        self.print1stat('capture_pointcount', self.pointcounts_grab, isInt=True)
        
    def print1stat(self, name, values, isInt=False):
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

def GrabberArgumentParser(*args, **kwargs):
    parser = argparse.ArgumentParser(*args, **kwargs)
    parser.add_argument("--kinect", action="store_true", help="View Azure Kinect camera in stead of realsense2 camera")
    parser.add_argument("--k4aoffline", action="store_true", help="View Azure Kinect pre-recorded files in stead of realsense2 camera")
    parser.add_argument("--synthetic", action="store_true", help="View synthetic pointcloud in stead of realsense2 camera")
    parser.add_argument("--proxy", type=int, action="store", metavar="PORT", help="View proxyser pointcloud in stead of realsense2 camera, proxyserver listens on PORT")
    parser.add_argument("--certh", action="store", metavar="URL", help="View Certh pointcloud in stead of realsense2 camera, captured from Rabbitmq server URL")
    parser.add_argument("--certh_data", action="store", metavar="NAME", help="Use NAME for certh data exchange (default: VolumetricData)", default="VolumetricData")
    parser.add_argument("--certh_metadata", action="store", metavar="NAME", help="Use NAME for certh metadata exchange (default: VolumetricMetaData)", default="VolumetricMetaData")
    parser.add_argument("--file", action="store", metavar="FILE", help="Continually show pointcloud from ply file FILE ")
    parser.add_argument("--dir", action="store", metavar="DIR", help="Continually show pointclouds from ply files in DIR in alphabetical order")
    parser.add_argument("--npoints", action="store", metavar="N", type=int, help="Limit number of points (approximately) in synthetic pointcoud", default=0)
    parser.add_argument("--dump", action="store_true", help="Playback .cwipcdump files in stead of .ply files with --file or --dump")
    parser.add_argument("--fps", action="store", type=int, help="Limit playback rate to FPS", default=0)
    return parser
    
def ArgumentParser(*args, **kwargs):
    parser = GrabberArgumentParser(*args, **kwargs)
    parser.add_argument("--count", type=int, action="store", metavar="N", help="Stop after receiving N pointclouds")
    parser.add_argument("--verbose", action="store_true", help="Print information about each pointcloud after it has been received")
    return parser