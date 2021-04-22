import sys
import os
import time
import threading
import time
import argparse
import traceback
import queue
import cwipc
from ._scriptsupport import *

class Visualizer:
    HELP="""
space         Pause/resume
mouse_left    Rotate viewpoint
mouse_scroll  Zoom in/out
mouse_right   Up/down viewpoint
0,1,2,4,8     Select single tile to view ( 0=All )
a             Show all tiles
w             Write PLY file
?,h           Help
q             Quit
    """
    
    def __init__(self, verbose=False):
        self.visualiser = None
        self.producer = None
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.cur_pc = None
        self.paused = False
        self.tilefilter = None
        self.start_window()
        
    def set_producer(self, producer):
        self.producer = producer    
        
    def run(self):
        while self.producer and self.producer.is_alive():
            try:
                pc = self.queue.get(timeout=0.033)
                if self.paused:
                    if pc: pc.free()
                    pc = None
                ok = self.draw_pc(pc)
                if not self.paused:
                    if self.cur_pc:
                        self.cur_pc.free()
                    self.cur_pc = pc
                if not ok: break
            except queue.Empty:
                pass
        
    def feed(self, pc):
        try:
            self.queue.put(pc, timeout=0.5)
        except queue.Full:
            pc.free()
            
    def start_window(self):
        cwd = os.getcwd()   # Workaround for cwipc_window changing working directory
        self.visualiser = cwipc.cwipc_window("cwipc_view")
        os.chdir(cwd)
        if self.verbose: print('display: started', flush=True)
        self.visualiser.feed(None, True)

    def draw_pc(self, pc):
        """Draw pointcloud"""
        if pc:
            pc_to_show = pc
            if self.tilefilter:
                pc_to_show = cwipc.cwipc_tilefilter(pc, self.tilefilter)
            ok = self.visualiser.feed(pc_to_show, True)
            if pc_to_show != pc:
                pc_to_show.free()
            if not ok: 
                print('display: window.feed() returned False')
                return False
        cmd = self.visualiser.interact(None, "?hq +-cwa012345678", 30) 
        if cmd == "q":
            return False
        elif cmd == '?' or cmd == 'h':
            print(Visualizer.HELP)
        elif cmd == "w" and self.cur_pc:
            filename = f'pointcloud_{self.cur_pc.timestamp()}.ply'
            cwipc.cwipc_write(filename, self.cur_pc)
            print(f'Saved as {filename} in {os.getcwd()}')
        elif cmd == " ":
            self.paused = not self.paused
        elif cmd == 'a':
            self.tilefilter = None
        elif cmd in '0123456789':
            if int(cmd) == 0:
                self.tilefilter = 0
                print("Showing all tiles")
            else:
                self.tilefilter = pow(2,int(cmd)-1)
                print("Showing tile =",self.tilefilter)
        elif cmd == '\0':
            pass
        else: #c to crash and print stack trace
            print(HELP)
        return True
    
def main():
    SetupStackDumper()
    parser = ArgumentParser(description="View pointcloud streams", epilog="Interactive commands:\n" + Visualizer.HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nodisplay", action="store_true", help="Don't display pointclouds, only prints statistics at the end")
    parser.add_argument("--savecwicpc", action="store", metavar="DIR", help="Save compressed pointclouds to DIR")
    args = parser.parse_args()
    #
    # Create source
    #
    source = cwipc_genericsource(args)

    if not args.nodisplay:
        visualizer = Visualizer(args.verbose)
    else:
        visualizer = None

    sourceServer = SourceServer(source, visualizer, count=args.count, verbose=args.verbose)
    sourceThread = threading.Thread(target=sourceServer.run, args=())
    if visualizer:
        visualizer.set_producer(sourceThread)

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
    
if __name__ == '__main__':
    main()
    
    
    
