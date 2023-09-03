import time
from typing import Union, List
from ..util import cwipc_crop, cwipc_wrapper

class CustomFilter:
    """
    crop - Remove points outside a given bounding box
        Arguments:
            minx: minimum X
            maxx: maximum X
            miny: minimum Y
            maxy: maximum Y
            minz: minimum Z
            maxz: maximum Z
    """
    def __init__(self, minx : float, maxx : float, miny : float, maxy : float, minz : float, maxz : float):
        self.bounding_box = (minx, maxx, miny, maxy, minz, maxz)
        self.count = 0
        self.times_crop = []
        self.original_pointcounts_crop = []
        self.pointcounts_crop = []
        
    def filter(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        self.count += 1
        t1_d = time.time()
        self.original_pointcounts_crop.append(pc.count())
        cropped_pc = cwipc_crop(pc, self.bounding_box)
        pc.free()
        pc = cropped_pc
        t2_d = time.time()
        self.times_crop.append(t2_d-t1_d)
        self.pointcounts_crop.append(pc.count())
        return pc

    def statistics(self) -> None:
        if self.times_crop:
            self.print1stat('duration', self.times_crop)
        if self.original_pointcounts_crop:
            self.print1stat('original_pointcount', self.original_pointcounts_crop, True)
        if self.pointcounts_crop:
            self.print1stat('pointcount', self.pointcounts_crop, True)

    def print1stat(self, name : str, values : Union[List[int], List[float]], isInt : bool=False) -> None:
        count = len(values)
        if count == 0:
            print(f'crop: {name}: count=0')
            return
        minValue = min(values)
        maxValue = max(values)
        avgValue = sum(values) / count
        if isInt:
            fmtstring = 'crop: {}: count={}, average={:.3f}, min={:d}, max={:d}'
        else:
            fmtstring = 'crop: {}: count={}, average={:.3f}, min={:.3f}, max={:.3f}'
        print(fmtstring.format(name, count, avgValue, minValue, maxValue))
