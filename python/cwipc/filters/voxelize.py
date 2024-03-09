import time
from typing import Union, List
from .abstract import cwipc_abstract_filter
from ..util import cwipc_downsample, cwipc_wrapper

class VoxelizeFilter(cwipc_abstract_filter):
    """
    voxelize - Reduce number of points by voxelization (combining points within a cube by their average)
        Arguments:
            vsize: a cube of vsize*vsize*vsize is used (float)
    """
    filtername = "voxelize"

    def __init__(self, vsize : float):
        self.vsize = vsize
        self.count = 0
        self.reduction_factor = 1.0
        self.times = []
        self.original_pointcounts = []
        self.pointcounts = []
        
    def filter(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        self.count += 1
        t1_d = time.time()
        self.original_pointcounts.append(pc.count())
        voxelized_pc = cwipc_downsample(pc, self.vsize)
        pc.free()
        pc = voxelized_pc
        t2_d = time.time()
        self.times.append(t2_d-t1_d)
        self.pointcounts.append(pc.count())
        return pc

    def statistics(self) -> None:
        if self.times:
            self.print1stat('duration', self.times)
        if self.original_pointcounts:
            self.print1stat('original_pointcount', self.original_pointcounts, True)
        if self.pointcounts:
            self.print1stat('pointcount', self.pointcounts, True)

    def print1stat(self, name : str, values : Union[List[int], List[float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print(f'{self.filtername}: {name}: count=0')
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = '{}: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = '{}: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(self.filtername, name, count, avgValue, minValue, maxValue))

CustomFilter = VoxelizeFilter
