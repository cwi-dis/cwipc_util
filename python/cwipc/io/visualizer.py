import os
from typing import Dict, Any
import time
import time
import argparse
import queue
from .. import cwipc_window, cwipc_tilefilter, cwipc_write, cwipc_wrapper
from ..net.abstract import *
import cv2
import numpy as np

class Visualizer(cwipc_sink_abstract):
    """Asynchronous point cloud viewer. The API is very similar to cwipc_window, but this class runs the UI
    in a separate thread so interaction (and feeding of the point clouds) does not cause the whole program to grind
    to a halt.
    """
    HELP="""
space         Pause/resume
.             Single step (for recordings)
<             Rewind (for recordings)
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
q,ESC         Quit
    """
    output_queue : queue.Queue[Optional[cwipc_wrapper]]

    def __init__(self, verbose=False, nodrop=False, args : Optional[argparse.Namespace]=None, title="cwipc_view", **kwargs):
        self.title = title
        self.visualiser = None
        self.producer = None
        self.source = None
        self.cameraconfig = None
        self.show_rgb = False
        self.rgb_cw = False
        self.rgb_ccw = False
        if args:
            self.cameraconfig = args.cameraconfig
            self.show_rgb = args.rgb
            self.rgb_cw = args.rgb_cw
            self.rgb_ccw = args.rgb_ccw
        for k in kwargs:
            # Should only be the ones that can also be in args, but hey...
            setattr(self, k, kwargs[k])
        self.output_queue = queue.Queue(maxsize=2)
        self.verbose = verbose
        self.cur_pc = None
        self.paused = False
        self.single_step = False
        self.nodrop = nodrop
        self.tilefilter = None
        self.filter_mode = 'mask'
        self.start_window()
        self.stopped = False
        self.stop_requested = False
        self.point_size_min = 0.0005
        self.point_size_power = 0
        self.display_fps = 30
        
    def statistics(self) -> None:
        pass

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_requested = True
    
    def set_producer(self, producer : cwipc_producer_abstract) -> None:
        self.producer = producer
        
    def set_source(self, source : cwipc_source_abstract) -> None:
        self.source = source
        
    def is_alive(self) -> bool:
        return not self.stopped  
        
    def _continue_running(self) -> bool:
        if self.stop_requested:
            return False
        if self.producer:
            return self.producer.is_alive()
        if self.source:
            return not self.source.eof()
        return False
    
    def run(self) -> None:
        while self._continue_running():
            try:
                if self.paused and self.nodrop:
                    ok = self.draw_pc(None)
                    if not ok:
                        break
                    continue
                get_timeout = (1.0 / self.display_fps)
                pc = self.output_queue.get(timeout=get_timeout)
                if self.paused:
                    if pc: pc.free()
                    pc = None
                ok = self.draw_pc(pc)
                if not ok: break
                if not self.paused:
                    if self.cur_pc:
                        self.cur_pc.free()
                    self.cur_pc = pc
                if self.single_step and pc != None:
                    print(f"single_step: ts={pc.timestamp()}")
                    self.paused = True
                    self.single_step = False
            except queue.Empty:
                pass
        if self.cur_pc:
            self.cur_pc.free()
            self.cur_pc = None
        self.stopped = True
        # Empty queue
        try:
            while True:
                pc = self.output_queue.get(block=False)
        except queue.Empty:
            pass
        self.nodrop = False
        if self.show_rgb:
            cv2.destroyWindow("RGB")
        if self.visualiser:
            self.visualiser.free()
        self.visualiser = None
        
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
        self.visualiser = cwipc_window(self.title)
        os.chdir(cwd)
        if self.verbose: print('display: started', flush=True)
        self.visualiser.feed(None, True)

    def draw_pc(self, pc : Optional[cwipc_wrapper]) -> bool:
        """Draw pointcloud and interact with the visualizer.
        If None is passed as the pointcloud it will only interact (keeping the previous pointcloud visible)
        """
        assert self.visualiser
        cellsize = self.point_size_min
        if pc:
            # First show RGB, if wanted
            if self.show_rgb:
                self.draw_rgb(pc)
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
        return self.interact_visualiser() # Note we pass the original pc to interact, not the modified pc.

    def interact_visualiser(self) -> bool:
        """Allow user interaction with the visualizer."""
        assert self.visualiser
        interaction_duration = 500 // self.display_fps
        cmd = self.visualiser.interact(None, "?h\x1bq .<+-cwamirsn0123456789", interaction_duration)
        if cmd == "q" or cmd == "\x1b":
            return False
        elif cmd == '?' or cmd == 'h':
            print(self.HELP)
        elif cmd == "w" and self.cur_pc:
            self.write_current_pointcloud()
        elif cmd == " ":
            self.paused = not self.paused
            if self.paused:
                if self.cur_pc:
                    print(f"pause: ts={self.cur_pc.timestamp()}")
        elif cmd == ".":
            self.single_step = True
            self.paused = False
            print("xxxjack single step requested")
        elif cmd == "<":
            if not self.source.seek(0):
                print("Input source does not support seek")

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
        elif cmd == '-':
            if self.point_size_power>0:
                self.point_size_power -= 1
        elif cmd == '\0':
            pass
        elif cmd == 'c':
            self.reload_cameraconfig()
        else:
            print(f"Unknown command {repr(cmd)}")
            print(self.HELP, flush=True)
        return True

    def write_current_pointcloud(self):
        """Save the current point cloud. May be overridden in subclasses to do something else."""
        assert self.cur_pc
        filename = f'pointcloud_{self.cur_pc.timestamp()}.ply'
        cwipc_write(filename, self.cur_pc, True) #writing in binary
        print(f'Saved as {filename} in {os.getcwd()}')

    def draw_rgb(self, pc : cwipc_wrapper) -> None:
        """Draw a window with the RGB data of all cameras."""
        auxdata = pc.access_auxiliary_data()
        if not auxdata:
            return
        per_camera_images = auxdata.get_all_images("rgb.")
        all_images = list(per_camera_images.values())

        if len(all_images) == 0:
            return
        if self.rgb_cw or self.rgb_ccw:
            full_image = cv2.hconcat(all_images)
        else:
            full_image = cv2.vconcat(all_images)
        # Scale to something reasonable
        h, w, _ = full_image.shape
        hscale = 1024 / h
        wscale = 1024 / w
        scale = min(hscale, wscale)
        if scale < 1:
            new_h = int(h*scale)
            new_w = int(w*scale)
            full_image = cv2.resize(full_image, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("RGB", full_image)
        cv2.waitKey(1)
    
    def reload_cameraconfig(self) -> None:
        """Reload the cameras. Call after changing cameraconfig.json."""
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
