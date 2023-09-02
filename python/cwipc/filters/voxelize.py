import time
from typing import Union, List
from ..util import cwipc_downsample, cwipc_wrapper

class CustomFilter:
    """
    voxelize - Reduce number of points by voxelization (combining points within a cube by their average)
        Arguments:
            vsize: a cube of vsize*vsize*vsize is used (float)
    """
    def __init__(self, vsize : float):
        self.vsize = vsize
        self.count = 0
        self.reduction_factor = 1.0
        self.times_voxelize = []
        self.original_pointcounts_voxelize = []
        self.pointcounts_voxelize = []
        
    def filter(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        self.count += 1
        t1_d = time.time()
        self.original_pointcounts_voxelize.append(pc.count())
        voxelized_pc = cwipc_downsample(pc, self.vsize)
        pc.free()
        pc = voxelized_pc
        t2_d = time.time()
        self.times_voxelize.append(t2_d-t1_d)
        self.pointcounts_voxelize.append(pc.count())
        return pc

    def statistics(self) -> None:
        if self.times_voxelize:
            self.print1stat('duration', self.times_voxelize)
        if self.original_pointcounts_voxelize:
            self.print1stat('original_pointcount', self.original_pointcounts_voxelize, True)
        if self.pointcounts_voxelize:
            self.print1stat('pointcount', self.pointcounts_voxelize, True)

    def print1stat(self, name : str, values : Union[List[int], List[float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print(f'voxelize: {name}: count=0')
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'voxelize: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'voxelize: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))
