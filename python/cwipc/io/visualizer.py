import os
import sys
from typing import Dict, Any
import time
import time
import argparse
import queue
from .. import cwipc_window, cwipc_tilefilter, cwipc_write, cwipc_wrapper
from ..net.abstract import *
from ..filters.abstract import cwipc_abstract_filter
from ..filters.colorize import ColorizeFilter
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
f             Colorize points to show contributing cameras
r             Toggle skeleton rendering (only if executed with --skeleton)
w             Write PLY file
t             Timelapse: like w but after a 5 second delay
p             Timelapse pause: pause after 5 seconds
c             Reload cameraconfig
?,h           Help
q,ESC         Quit
    """
    output_queue : queue.Queue[Optional[cwipc_wrapper]]
    display_filter : Optional[cwipc_abstract_filter]

    def __init__(self, verbose=False, nodrop=False, args : Optional[argparse.Namespace]=None, title="cwipc_view", **kwargs):
        self.title = title
        self.visualiser = None
        self.producer = None
        self.source = None
        self.cameraconfig = None
        self.show_rgb = False
        self.rgb_cw = False
        self.rgb_ccw = False
        self.rgb_full = False
        self.timestamps = False
        self.paused = False
        self.single_step = False
        self.redraw_requested = False
        self.display_filter = None
        if args:
            self.cameraconfig = args.cameraconfig
            self.show_rgb = args.rgb
            self.rgb_full = args.rgb_full
            self.rgb_cw = args.rgb_cw
            self.rgb_ccw = args.rgb_ccw
            if 'timestamp' in args:
                self.timestamps = args.timestamps
            # If we want paused we actually set single_step (so we get the first point cloud)
            if 'paused' in args and args.paused:
                self.paused = True
                self.single_step = True
        for k in kwargs:
            # Should only be the ones that can also be in args, but hey...
            setattr(self, k, kwargs[k])
        queue_size = 1 if self.timestamps else 2
        self.output_queue = queue.Queue(maxsize=queue_size)
        self.verbose = verbose
        self.cur_pc = None
        self.nodrop = nodrop
        self.tilefilter = None
        self.filter_mode = 'mask'
        self.start_window()
        self.stopped = False
        self.stop_requested = False
        self.point_size_min = 0.0005
        self.point_size_power = 0
        self.display_fps = 30
        self.display_filter = None
        self.timelapse_write_at = 0
        self.timelapse_beep_at = 0
        self.timelapse_pause_at = 0
        
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
                    self.draw_pc(None)
                    continue
                get_timeout = (1.0 / self.display_fps)
                pc = self.output_queue.get(timeout=get_timeout)
                if self.paused:
                    if pc: pc.free()
                    pc = None
                self.draw_pc(pc)
                if not self.paused:
                    if self.cur_pc:
                        self.cur_pc.free()
                    self.cur_pc = pc
                if self.single_step and pc != None:
                    self._show_timestamps(pc, 'single_step')
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
            if self.rgb_full:
                cv2.destroyAllWindows()
            else:
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

    def _show_timestamps(self, pc: cwipc_wrapper, label : str) -> None:
        print(f'{label}: ts={pc.timestamp()}')
        auxdata = pc.access_auxiliary_data()
        if auxdata != None and auxdata.count() > 0:
            for i in range(auxdata.count()):
                auxname = auxdata.name(i)
                if not "timestamps" in auxname:
                    continue
                descr = auxdata.description(i)
                print(f'{label}:    {auxname}: {descr}')

    def draw_pc(self, pc : Optional[cwipc_wrapper]) -> None:
        """Draw pointcloud and interact with the visualizer.
        If None is passed as the pointcloud it will only interact (keeping the previous pointcloud visible)
        """
        assert self.visualiser
        cellsize = self.point_size_min
        if self.redraw_requested:
            self.redraw_requested = False
            if pc == None:
                pc = self.cur_pc
        if pc:
            if self.timestamps:
                self._show_timestamps(pc, "timestamps")

            # First show RGB, if wanted
            if self.show_rgb:
                self.draw_rgb(pc)
            cellsize = max(pc.cellsize(), cellsize)
            pc._set_cellsize(cellsize*pow(2,self.point_size_power))
            pc_to_show = pc
            if self.verbose:
                if not self.paused:
                    print(f'display: showing pointcloud timestamp={pc.timestamp()} cellsize={pc.cellsize()} latency={time.time() - pc.timestamp()/1000.0:.3f}')
            if self.display_filter:
                pc_to_show = self.display_filter.filter(pc_to_show)
            if self.tilefilter:
                pc_to_show = cwipc_tilefilter(pc_to_show, self.tilefilter)
                if self.verbose:
                    print(f'display: selected {pc_to_show.count()} of {pc.count()} points')
            ok = self.visualiser.feed(pc_to_show, True)
            if pc_to_show != pc:
                pc_to_show.free()
            if not ok: 
                print('display: window.feed() returned False')
                self.stop()
                return
        self.interact_visualiser() # Note we pass the original pc to interact, not the modified pc.

    def interact_visualiser(self) -> None:
        """Allow user interaction with the visualizer."""
        assert self.visualiser
        interaction_duration = 500 // self.display_fps
        cmd = self.visualiser.interact(None, "?h\x1bq .<+-cfwtpamirsn0123456789", interaction_duration)
        # First handle the timelapse
        if self.timelapse_write_at > 0:
            now = time.time()
            if now >= self.timelapse_write_at:
                print(f"timelapse: Capture point cloud.\x07", file=sys.stderr)
                self.timelapse_beep_at = 0
                self.timelapse_write_at = 0
                self.write_current_pointcloud()
            if now >= self.timelapse_beep_at:
                print(f"timelapse: {int(self.timelapse_write_at - now)}\x07", file=sys.stderr)
                self.timelapse_beep_at += 1
        if self.timelapse_pause_at > 0:
            now = time.time()
            if now >= self.timelapse_pause_at:
                print(f"timelapse: pause", file=sys.stderr)
                self.paused = True
                self.timelapse_pause_at = 0
                if self.cur_pc:
                    self._show_timestamps(self.cur_pc, 'pause')
        # Now handle the commands
        if cmd == "q" or cmd == "\x1b":
            self.stop()
            return
        elif cmd == '?' or cmd == 'h':
            print(self.HELP)
        elif cmd == "w" and self.cur_pc:
            self.write_current_pointcloud()
        elif cmd == "t":
            now = time.time()
            self.timelapse_beep_at = now + 1
            self.timelapse_write_at = now + 5
            self.paused = False
            print(f"timelapse: capture in 5 seconds", file=sys.stderr)
        elif cmd == "p":
            now = time.time()
            self.timelapse_pause_at = now + 5
            print(f"timelapse: pause in 5 seconds", file=sys.stderr)
            self.paused = False
        elif cmd == " ":
            self.paused = not self.paused
            if self.paused:
                if self.cur_pc:
                    self._show_timestamps(self.cur_pc, 'pause')
        elif cmd == ".":
            self.single_step = True
            self.paused = False
        elif cmd == "<":
            if not self.source.seek(0):
                print("Input source does not support seek")
            self.paused = False
        elif cmd == 'a':
            self.select_tile_or_stream(all=True)
            self.redraw_requested = True
        elif cmd == 'm':
            self.select_mode('mask')
        elif cmd == 'i':
            self.select_mode('index')
        elif cmd == 's':
            self.select_mode('stream')
        elif cmd == 'n':
            self.select_tile_or_stream(increment=True)
            self.redraw_requested = True
        elif cmd == 'r':
            pass # toggle skeleton rendering (managed on cwipc_view)
        elif cmd in '0123456789':
            self.select_tile_or_stream(number=int(cmd))
            self.redraw_requested = True
        elif cmd == '+':
            self.point_size_power += 1
            self.redraw_requested = True
        elif cmd == '-':
            if self.point_size_power>0:
                self.point_size_power -= 1
                self.redraw_requested = True
        elif cmd == '\0':
            pass
        elif cmd == 'c':
            self.paused = False
            print("reload: reloading cameraconfig.json...", file=sys.stderr)
            self.reload_cameraconfig()
            print("reload: reloaded cameraconfig.json.", file=sys.stderr)
        elif cmd == 'f':
            if self.display_filter == None:
                self.display_filter = ColorizeFilter(0.8, "camera")
            else:
                self.display_filter = None
            self.redraw_requested = True
        else:
            print(f"Unknown command {repr(cmd)}")
            print(self.HELP, flush=True)
    
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
        if self.rgb_full:
            for name, image in per_camera_images.items():
                cv2.imshow(name, image)
            cv2.waitKey(1)
            return
        # Combine images into a single window to show
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
            full_image = cv2.resize(full_image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
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
