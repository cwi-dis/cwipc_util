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
0-9           Select single tile/stream to view ( 0=All )
n             Select next tile/stream to view
a             Show all tiles/streams
m             Toggle tile/stream selection tile mask mode
i             Toggle tile/stream selection tile index mode
s             Toggle tile/stream selection stream mode
w             Write PLY file
?,h           Help
q             Quit
    """
    
    def __init__(self, verbose=False, nodrop=False):
        self.visualiser = None
        self.producer = None
        self.source = None
        self.queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.cur_pc = None
        self.paused = False
        self.nodrop = nodrop
        self.tilefilter = None
        self.filter_mode = 'mask'
        self.start_window()
        self.stopped = False
        self.point_size_inc = 0.0
        
    def set_producer(self, producer):
        self.producer = producer
        
    def set_source(self, source):
        self.source = source
        
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
        cmd = self.visualiser.interact(None, "?hq +-cwamisn0123456789", 30) 
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
            self.select_tile_or_stream(all=True)
        elif cmd == 'm':
            self.select_mode('mask')
        elif cmd == 'i':
            self.select_mode('index')
        elif cmd == 's':
            self.select_mode('stream')
        elif cmd == 'n':
            self.select_tile_or_stream(increment=True)
        elif cmd in '0123456789':
            self.select_tile_or_stream(number=int(cmd))
        elif cmd == '+':
            self.point_size_inc += float(0.001)
        elif cmd == '-':
            self.point_size_inc -= float(0.001)
        elif cmd == '\0':
            pass
        else: #c to crash and print stack trace
            print(HELP, flush=True)
        return True
        
    def select_mode(self, newmode):
        self.filter_mode = newmode
        print(f"tilefilter mask mode: {self.filter_mode}. Showing all tiles", flush=True)
        self.tilefilter = None
        self.select_tile_or_stream(all=True)
        
    def select_tile_or_stream(self, *, number=None, all=False, increment=False):
        if self.filter_mode == 'stream':
            if not hasattr(self.source, 'select_stream'):
                print('Input does not support stream selection')
                return
            if all:
                number = 0
            if number == None:
                print('Network input only supports numeric stream selection')
                return
            ok = self.source.select_stream(number)
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
                print(f"Showing tile number {self.tilefilter} mask 0x{self.tilefilter:x}", flush=True)
            else:
                if number == 0:
                    self.tilefilter = 0
                    print("Showing all tiles", flush=True)
                else:
                    if self.filter_mode == 'mask':
                        self.tilefilter = pow(2,number-1)
                    else:
                        self.tilefilter = number
                    print(f"Showing tile number {self.tilefilter} mask 0x{self.tilefilter:x}", flush=True)

def main():
    SetupStackDumper()
    parser = ArgumentParser(description="View pointcloud streams", epilog="Interactive commands:\n" + Visualizer.HELP, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nodisplay", action="store_true", help="Don't display pointclouds, only prints statistics at the end")
    args = parser.parse_args()
    beginOfRun(args)
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    if not args.nodisplay:
        visualizer = Visualizer(args.verbose, nodrop=args.nodrop)
    else:
        visualizer = None

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
    
    
    
