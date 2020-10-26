import sys
import os
import signal
import threading
import time
import argparse
import traceback
import queue
import socket
import struct
import cwipc
import cwipc.codec
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
import cwipc.playback

# Convoluted code warning: adding ../python directory to path so we can import subsource
_sourcedir = os.path.dirname(__file__)
_sourcedir = os.path.realpath(_sourcedir)
_pardir = os.path.dirname(_sourcedir)
_pythondir = os.path.join(_pardir, 'python')
sys.path.append(_pythondir)

def _dump_app_stacks(*args):
    print("pc_echo: QUIT received, dumping all stacks, %d threads:" % len(sys._current_frames()), file=sys.stderr)
    for threadId, stack in list(sys._current_frames().items()):
        print("\nThreadID:", threadId, file=sys.stderr)
        traceback.print_stack(stack, file=sys.stderr)
        print(file=sys.stderr)

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
        header = struct.pack("<iiqfi", cwipc.CWIPC_POINT_PACKETHEADER_MAGIC, len(data), timestamp, cellsize, 0)
        x = self.socket.send(header)
        y = self.socket.send(data)

        
class SourceServer:
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
        pc = self.grabber.get()
        self.lastGrabTime = time.time()
        return pc
        
    def run(self):
        if self.verbose: print('grab: started', flush=True)
        while not self.stopped and not self.grabber.eof():
            t0 = time.time()
            pc = self.grab_pc()
            if not pc:
                print('grab: pointcloud==None')
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

def main():
    global ISSUE_20
    if hasattr(signal, 'SIGQUIT'):
        signal.signal(signal.SIGQUIT, _dump_app_stacks)
    parser = argparse.ArgumentParser(description="Send pointcloud stream to cwipc_proxy")
    parser.add_argument("--kinect", action="store_true", help="Send Azure Kinect camera in stead of realsense2 camera")
    parser.add_argument("--synthetic", action="store_true", help="Send synthetic pointcloud in stead of realsense2 camera")
    parser.add_argument("--proxy", type=int, action="store", metavar="PORT", help="Send proxyser pointcloud in stead of realsense2 camera, proxyserver listens on PORT")
    parser.add_argument("--certh", action="store", metavar="URL", help="Send Certh pointcloud in stead of realsense2 camera, captured from Rabbitmq server URL")
    parser.add_argument("--data", action="store", metavar="NAME", help="Use NAME for certh data exchange (default: VolumetricData)", default="VolumetricData")
    parser.add_argument("--metadata", action="store", metavar="NAME", help="Use NAME for certh metadata exchange (default: VolumetricMetaData)", default="VolumetricMetaData")
    parser.add_argument("--file", action="store", metavar="FILE", help="Continually send pointcloud from ply file FILE ")
    parser.add_argument("--dir", action="store", metavar="DIR", help="Continually send pointclouds from ply files in DIR in alphabetical order")
    parser.add_argument("--dump", action="store_true", help="Playback .cwipcdump files in stead of .ply files with --file or --dump")
    parser.add_argument("--fps", action="store", type=int, help="Limit playback rate to FPS")
    parser.add_argument("--count", type=int, action="store", metavar="N", help="Stop after receiving N pointclouds")
    parser.add_argument("--verbose", action="store_true", help="Print information about each pointcloud after it has been received")
    parser.add_argument("host", action="store", help="Hostname where cwipc_proxy server is running")
    parser.add_argument("port", type=int, action="store", help="Port where cwipc_proxy server is running")
    args = parser.parse_args()
    #
    # Create source
    #
    if args.kinect:
        if cwipc.kinect == None:
            print(f"{sys.argv[0]}: No support for Kinect grabber on this platform")
            sys.exit(-1)
        source = cwipc.kinect.cwipc_kinect()
    elif args.synthetic:
        source = cwipc.cwipc_synthetic()
    elif args.proxy:
        source = cwipc.cwipc_proxy('', args.proxy)
    elif args.certh:
        if cwipc.certh == None:
            print(f"{sys.argv[0]}: No support for CERTH grabber on this platform")
            sys.exit(-1)
        source = cwipc.certh.cwipc_certh(args.certh, args.data, args.metadata)
    elif args.file:
        source = cwipc.playback.cwipc_playback([args.file], ply=not args.dump, fps=args.fps, loop=True)
    elif args.dir:
        source = cwipc.playback.cwipc_playback(args.dir, ply=not args.dump, fps=args.fps, loop=True)
    else:
        if cwipc.realsense2 == None:
            print(f"{sys.argv[0]}: No support for realsense grabber on this platform")
            sys.exit(-1)
        source = cwipc.realsense2.cwipc_realsense2()

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
    
    
    
