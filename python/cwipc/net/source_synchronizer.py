import time
import os
import socket
import threading
import queue
from typing import Optional, List, Union
from .abstract import cwipc_source_abstract, cwipc_abstract, cwipc_rawsource_abstract
from ..util import cwipc_join, cwipc_wrapper


class _Synchronizer(threading.Thread, cwipc_source_abstract):
    """A source that combines point clouds gotten from multiple (tiled) sources into one stream."""
    
    QUEUE_WAIT_TIMEOUT=1
    output_queue : queue.Queue[Optional[cwipc_abstract]]

    def __init__(self, sources : List[cwipc_source_abstract], verbose : bool=False):
        threading.Thread.__init__(self)
        self.name = 'cwipc_util._NetDecoder'
        self.sources : List[cwipc_source_abstract] = sources
        self.n_tile = len(self.sources)
        self.input_buffers : List[Optional[cwipc_abstract]] = [None] * self.n_tile
        self.running = False
        self.verbose = verbose
        self.verbose = True
        self.output_queue = queue.Queue(maxsize=2)
        self.prefer_partial_over_unsynced = True
        #self.streamNumber = None
        self.combine_times = []
        self.late_per_occurrence = []
        self.desync_per_occurrence = []
        self.missing_per_occurrence = []

        
    def free(self) -> None:
        pass
        
    def start(self) -> None:
        assert not self.running
        if self.verbose: print('synchronizer: start', flush=True)
        self.running = True
        for s in self.sources:
            s.start()
        threading.Thread.start(self)
        
    def stop(self) -> None:
        if self.verbose: print('synchronizer: stop', flush=True)
        self.running = False
        for s in self.sources:
            s.stop()
        self.output_queue.put(None)
        self.join()
        
    def eof(self) -> bool:
        if not self.running:
            return True
        if not self.output_queue.empty():
            return False
        return self._any_source_eof()
    
    def _any_source_eof(self) -> bool:
        for s in self.sources:
            if s.eof():
                return True
        return False
    
    def available(self, wait : bool=False) -> bool:
        # xxxjack if wait==True should get and put
        if not self.running:
            return False
        if not self.output_queue.empty():
            return True
        for s in self.sources:
            if not s.available():
                return False
        return True
        
    def get(self) -> Optional[cwipc_abstract]:
        if self.eof():
            return None
        pc = self.output_queue.get()
        return pc

    def _select_stream(self, streamIndex : int) -> None:
        if self.verbose: print(f'synchronizer: select_stream({streamIndex}', flush=True)
        return self.source.enable_stream(self.tileNum, streamIndex)
        
    def run(self) -> None:
        if self.verbose: print(f"synchronizer: thread started", flush=True)
        earliest_timestamp = 0
        latest_timestamp = 0
        while self.running:
            if self._any_source_eof():
                break
            any_work_done = False
            # Compute the earliest timestamp (which is the latest of the currently available buffer heads)
            for buffer_head in self.input_buffers:
                if buffer_head:
                    ts = buffer_head.timestamp()
                    latest_timestamp = max(latest_timestamp, ts)
            # Throw away outdated heads
            for i in range(self.n_tile):
                if self.input_buffers[i] and self.input_buffers[i].timestamp() < earliest_timestamp:
                    print(f"synchronizer: free input_buffer[{i}]")
                    self.input_buffers[i].free()
                    self.input_buffers[i] = None
            # See whether any empty buffer slots can be filled, and see whether any empty ones remain
            any_empty_input_buffers = False
            for i in range(self.n_tile):
                if self.input_buffers[i] == None:
                    if self.sources[i].available():
                        pc = self.sources[i].get()
                        if not pc:
                            print(f"synchronizer: source {i} returned no point cloud")
                            any_empty_input_buffers = True
                            break
                        if self.verbose:
                            print(f"synchronizer: got ts={pc.timestamp()} from {i}")
                        if pc.timestamp() >= earliest_timestamp:
                            self.input_buffers[i] = pc
                            any_work_done = True
                        else:
                            # Unfortunately this partial point cloud is too late
                            too_late = earliest_timestamp - pc.timestamp()
                            if self.verbose: print(f"synchronizer: tile {i}: too late by {too_late}")
                            pc.free()
                            self.late_per_occurrence.append(too_late)
                    else:
                        any_empty_input_buffers = True
            # If not all buffers are filled yet we wait, and go through the loop once more.
            if any_empty_input_buffers:
                if self.verbose: print(f"synchronizer: still missing pointclouds")
                time.sleep(0.001)
                continue
            current_timestamps = [pc.timestamp() for pc in self.input_buffers]
            if self.verbose: print(f"synchronizer: all tiles pc={current_timestamps}")
            current_earliest_timestamp = min(current_timestamps)
            current_latest_timestamp = max(current_timestamps)
            # If we are okay with producing partial point clouds we keep only
            # the ones with the correct timestamp
            # The alternative is that we always produce a full point cloud, but
            # it may be de-synced (tiles with different timestamps)
            if self.prefer_partial_over_unsynced:
                to_combine = [pc for pc in self.input_buffers if pc.timestamp() == current_earliest_timestamp]
                desync = 0
            else:
                to_combine = [pc for pc in self.input_buffers]
                desync = current_latest_timestamp - current_earliest_timestamp
            if len(to_combine) < self.n_tile:
                missing = self.n_tile - len(to_combine)
                self.missing_per_occurrence.append(missing)
            if desync > 0:
                self.desync_per_occurrence.append(desync)
            # Now we just need to combine the point clouds, and set a reasonable timestamp and cellsize
            result_pc : Optional[cwipc_wrapper] = None
            t0 = time.time()
            current_cellsize = min([pc.cellsize() for pc in to_combine])
            must_free_old_result = False
            for pc in to_combine:
                assert pc
                if result_pc == None:
                    result_pc = pc
                else:
                    assert(result_pc)
                    new_result_pc = cwipc_join(result_pc, pc)
                    if must_free_old_result:
                        result_pc.free()
                    must_free_old_result = True
                    # Don't do this, may be reused later: pc.free()
                    result_pc = new_result_pc
            t1 = time.time()
            result_pc._set_timestamp(current_earliest_timestamp)
            result_pc._set_cellsize(current_cellsize)
            earliest_timestamp = current_earliest_timestamp + 1
            self.combine_times.append(t1-t0)
            self.output_queue.put(result_pc)

            if self.verbose: print(f'synchronizer: produced pointcloud ts={result_pc.timestamp()} with {result_pc.count()} points', flush=True)
        if self.verbose: print(f"synchronizer: thread exiting", flush=True)


    def statistics(self) -> None:
        self.print1stat('combine_time', self.combine_times)
        self.print1stat('late', self.late_per_occurrence)
        self.print1stat('desync', self.desync_per_occurrence)
        self.print1stat('missing', self.missing_per_occurrence, isInt=True)

        for s in self.sources:
            if hasattr(s, 'statistics'):
                s.statistics()
        
    def print1stat(self, name : str, values : List[Union[int, float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print('netdecoder: {}: count=0'.format(name))
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'netdecoder: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'netdecoder: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))

    def request_auxiliary_data(self, name: str) -> None:
        assert False

    def auxiliary_data_requested(self, name: str) -> bool:
        return False
    
def cwipc_source_synchronizer(sources : List[cwipc_source_abstract], verbose : bool=False) -> cwipc_source_abstract:
    """Return cwipc_source-like object that combines and synchronizes pointclouds from multiple sources"""
    rv = _Synchronizer(sources, verbose=verbose)
    return rv
        
