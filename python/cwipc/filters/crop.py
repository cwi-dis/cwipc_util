import time
from typing import Union, List
from ..util import cwipc_crop, cwipc_wrapper

class CropFilter:
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
    filtername = "crop"
    
    def __init__(self, minx : float, maxx : float, miny : float, maxy : float, minz : float, maxz : float):
        self.bounding_box = (minx, maxx, miny, maxy, minz, maxz)
        self.count = 0
        self.times = []
        self.original_pointcounts = []
        self.pointcounts = []
        
    def filter(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        self.count += 1
        t1_d = time.time()
        self.original_pointcounts.append(pc.count())
        cropped_pc = cwipc_crop(pc, self.bounding_box)
        pc.free()
        pc = cropped_pc
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

CustomFilter = CropFilter
