import time
from typing import Union, List
from ..util import cwipc_wrapper, cwipc_from_points

class TransformFilter:
    """
    transform - Adjust coordinate system of the point clouds.
        Arguments:
            x: offset to add to X coordinates
            y: offset to add to Y coordinates
            z: offset to add to Z coordinates
            scale: scale factor to apply (after the offsets)
        The analyze filter will give an educated guess for the parameters, for point clouds of humans.
    """
    filtername = "transform"

    def __init__(self, x : float, y: float, z : float, scale : float):
        self.x = x
        self.y = y
        self.z = z
        self.scale = scale
        self.count = 0
        self.times = []
        
        
    def filter(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        """xxxjack this method should be rewritten using numpy"""
        self.count += 1
        t1_d = time.time()
        points = pc.get_points()
        timestamp = pc.timestamp()
        cellsize = pc.cellsize()
        newpoints = []
        for p in points:
            p.x = (p.x + self.x) * self.scale
            p.y = (p.y + self.y) * self.scale
            p.z = (p.z + self.z) * self.scale
            newpoints.append(p)
        newpc = cwipc_from_points(newpoints, timestamp)
        newpc._set_cellsize(cellsize * self.scale)
        pc.free()
        t2_d = time.time()
        self.times.append(t2_d-t1_d)
        return newpc
        
    def statistics(self):
        print(f"transform: count={self.count}")
        if self.times:
            self.print1stat('duration', self.times)


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

CustomFilter = TransformFilter
