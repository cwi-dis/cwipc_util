import os
import time
import threading
import time
import argparse
import traceback
import queue
from .. import cwipc_window, cwipc_tilefilter, cwipc_write, cwipc_wrapper
from ..net.abstract import *

class Visualizer(cwipc_sink_abstract):
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
r             Toggle skeleton rendering (only if executed with --skeleton)
w             Write PLY file
c             Reload cameraconfig
?,h           Help
q             Quit
    """
    output_queue : queue.Queue[Optional[cwipc_wrapper]]

    def __init__(self, verbose=False, nodrop=False, args : Optional[argparse.Namespace]=None):
        self.visualiser = None
        self.producer = None
        self.source = None
        self.cameraconfig = None
        if args:
            self.cameraconfig = args.cameraconfig
        self.output_queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.cur_pc = None
        self.paused = False
        self.nodrop = nodrop
        self.tilefilter = None
        self.filter_mode = 'mask'
        self.start_window()
        self.stopped = False
        self.point_size_min = 0.0005
        self.point_size_power = 0
        
    def statistics(self) -> None:
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass
    
    def set_producer(self, producer : cwipc_producer_abstract) -> None:
        self.producer = producer
        
    def set_source(self, source : cwipc_source_abstract) -> None:
        self.source = source
        
    def is_alive(self) -> bool:
        return not self.stopped  
        
    def run(self) -> None:
        while self.producer and self.producer.is_alive():
            try:
                if self.paused and self.nodrop:
                    ok = self.draw_pc(None)
                    continue
                pc = self.output_queue.get(timeout=0.033)
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
        
    def feed(self, pc : cwipc_wrapper) -> None:
        try:
            if self.nodrop:
                self.output_queue.put(pc)
            else:
                self.output_queue.put(pc, timeout=0.5)
        except queue.Full:
            pc.free()
            
    def start_window(self):
        cwd = os.getcwd()   # Workaround for cwipc_window changing working directory
        self.visualiser = cwipc_window("cwipc_view")
        os.chdir(cwd)
        if self.verbose: print('display: started', flush=True)
        self.visualiser.feed(None, True)

    def draw_pc(self, pc : Optional[cwipc_wrapper]) -> bool:
        """Draw pointcloud"""
        assert self.visualiser               
        cellsize = self.point_size_min
        if pc:
            cellsize = max(pc.cellsize(), cellsize)
            pc._set_cellsize(cellsize*pow(2,self.point_size_power))
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
        cmd = self.visualiser.interact(None, "?hq +-cwamirsn0123456789", 30)
        if cmd == "q":
            return False
        elif cmd == '?' or cmd == 'h':
            print(self.HELP)
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
        elif cmd == 'r':
            pass # toggle skeleton rendering (managed on cwipc_view)
        elif cmd in '0123456789':
            self.select_tile_or_stream(number=int(cmd))
        elif cmd == '+':
            self.point_size_power += 1
            print(f'Changed point size = {cellsize*pow(2,self.point_size_power)}')
        elif cmd == '-':
            if self.point_size_power>0:
                self.point_size_power -= 1
                print(f'Changed point size = {cellsize*pow(2,self.point_size_power)}')
            else:
                print(f'Reached point size min = {cellsize*pow(2,self.point_size_power)}')
        elif cmd == '\0':
            pass
        elif cmd == 'c':
            self.reload_cameraconfig()
        else: #c to crash and print stack trace
            print(self.HELP, flush=True)
        return True
    
    def reload_cameraconfig(self) -> None:
        assert self.source
        assert hasattr(self.source, "reload_config")
        try:
            ok = self.source.reload_config(self.cameraconfig) # type: ignore
            if not ok:
                print("reload_cameraconfig: failed to reload cameraconfig")
        except Exception as e:
            print(f"reload_cameraconfig: Exception: {e}")
    
    def select_mode(self, newmode):
        self.filter_mode = newmode
        print(f"tilefilter mask mode: {self.filter_mode}. Showing all tiles", flush=True)
        self.tilefilter = None
        self.select_tile_or_stream(all=True)
        
    def select_tile_or_stream(self, *, number : Optional[int]=None, all=False, increment=False):
        assert self.source
        if self.filter_mode == 'stream':
            if not hasattr(self.source, 'select_stream'):
                print('Input does not support stream selection')
                return
            if all:
                number = 0
            if number == None:
                print('Network input only supports numeric stream selection')
                return
            ok = self.source.select_stream(number) # type: ignore
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
                assert number != None
                if number == 0:
                    self.tilefilter = 0
                    print("Showing all tiles", flush=True)
                else:
                    if self.filter_mode == 'mask':
                        self.tilefilter = pow(2,number-1)
                    else:
                        self.tilefilter = number
                    print(f"Showing tile number {self.tilefilter} mask 0x{self.tilefilter:x}", flush=True)
