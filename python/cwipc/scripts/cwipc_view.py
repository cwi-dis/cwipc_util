import sys
import os
import time
import threading
import time
import argparse
import traceback
import queue
from .. import cwipc_window, cwipc_tilefilter, cwipc_write
from ._scriptsupport import *

class Visualizer:
    HELP="""
space         Pause/resume
mouse_left    Rotate viewpoint
mouse_scroll  Zoom in/out
mouse_right   Up/down viewpoint
+/-           Increase/decrease point size
0-9           Select single tile to view ( 0=All )
n             Select next tile to view
a             Show all tiles
m             Toggle tile selection mask/index mode
w             Write PLY file
?,h           Help
q             Quit
    """
    
    def __init__(self, verbose=False, nodrop=False):
        self.visualiser = None
        self.producer = None
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.cur_pc = None
        self.paused = False
        self.nodrop = nodrop
        self.tilefilter = None
        self.tilefilter_mask = True
        self.start_window()
        self.stopped = False
        self.point_size_inc = 0.0
        
    def set_producer(self, producer):
        self.producer = producer
        
    def is_alive(self):
        return not self.stopped  
        
    def run(self):
        while self.producer and self.producer.is_alive():
            try:
                if self.paused and self.nodrop:
                    ok = self.draw_pc(None)
                    continue
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
        self.stopped = True
        
    def feed(self, pc):
        try:
            if self.nodrop:
                self.queue.put(pc)
            else:
                self.queue.put(pc, timeout=0.5)
        except queue.Full:
            pc.free()
            
    def start_window(self):
        cwd = os.getcwd()   # Workaround for cwipc_window changing working directory
        self.visualiser = cwipc_window("cwipc_view")
        os.chdir(cwd)
        if self.verbose: print('display: started', flush=True)
        self.visualiser.feed(None, True)

    def draw_pc(self, pc):
        """Draw pointcloud"""
        if pc:
            pc._set_cellsize(pc.cellsize()+self.point_size_inc)
            pc_to_show = pc
            if self.verbose:
                if not self.paused:
                    print(f'display: showing pointcloud timestamp={pc.timestamp()} cellsize={pc.cellsize()} latency={time.time() - pc.timestamp()/1000.0:.3f}')
            if self.tilefilter:
                pc_to_show = cwipc_tilefilter(pc, self.tilefilter)
                if self.verbose:
                    print(f'display: selected {pc_to_show.count()} of {pc.count()} points')                  
            ok = self.visualiser.feed(pc_to_show, True)
            if pc_to_show != pc:
                pc_to_show.free()
            if not ok: 
                print('display: window.feed() returned False')
                return False
        cmd = self.visualiser.interact(None, "?hq +-cwamn0123456789", 30) 
        if cmd == "q":
            return False
        elif cmd == '?' or cmd == 'h':
            print(Visualizer.HELP)
        elif cmd == "w" and self.cur_pc:
            filename = f'pointcloud_{self.cur_pc.timestamp()}.ply'
            cwipc_write(filename, self.cur_pc, True) #writing in binary
            print(f'Saved as {filename} in {os.getcwd()}')
        elif cmd == " ":
            self.paused = not self.paused
        elif cmd == 'a':
            self.select_tile(all=True)
        elif cmd == 'm':
            self.tilefilter_mask = not self.tilefilter_mask
            print(f"tilefilter mask mode: {self.tilefilter_mask}. Showing all tiles", flush=True)
            self.select_tile(all=True)
        elif cmd == 'n':
            self.select_tile(increment=True)
        elif cmd in '0123456789':
            self.select_tile(number=int(cmd))
        elif cmd == '+':
            self.point_size_inc += float(0.001)
        elif cmd == '-':
            self.point_size_inc -= float(0.001)
        elif cmd == '\0':
            pass
        else: #c to crash and print stack trace
            print(HELP, flush=True)
        return True
        
    def select_tile(self, *, number=None, all=False, increment=False):
        print(f'xxxjack producer={self.producer}')
        if hasattr(self.producer, 'select_stream'):
            if number == None or all or increment:
                print('Network input only supports numeric stream selection')
                return
            ok = self.producer.select_stream(number)
            if ok:
                print(f'Selecting input stream {number}')
            else:
                print(f'Error selecting input stream {number}, probably non-existent')
        else:
            if all:
                self.tilefilter = None
                print("Showing all tiles")
            elif increment:
                if not self.tilefilter:
                    self.tilefilter = 1
                else:
                    self.tilefilter = self.tilefilter + 1
                print(f"Showing tile {self.tilefilter} 0x{self.tilefilter:x}", flush=True)
            else:
                if number == 0:
                    self.tilefilter = 0
                    print("Showing all tiles", flush=True)
                else:
                    if self.tilefilter_mask:
                        self.tilefilter = pow(2,number-1)
                    else:
                        self.tilefilter = number
                    print(f"Showing tile {self.tilefilter} 0x{self.tilefilter:x}", flush=True)

def main():
    SetupStackDumper()
    parser = ArgumentParser(description="View pointcloud streams", epilog="Interactive commands:\n" + Visualizer.HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nodisplay", action="store_true", help="Don't display pointclouds, only prints statistics at the end")
    args = parser.parse_args()
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    if not args.nodisplay:
        visualizer = Visualizer(args.verbose, nodrop=args.nodrop)
    else:
        visualizer = None

    sourceServer = SourceServer(source, visualizer, count=args.count, inpoint=args.inpoint, outpoint=args.outpoint, verbose=args.verbose, source_name=source_name)
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
    
    
    
