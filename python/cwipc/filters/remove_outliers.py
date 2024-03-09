import time
from typing import Union, List
from .abstract import cwipc_abstract_filter
from ..util import cwipc_remove_outliers, cwipc_wrapper

class RemoveOutliersFilter(cwipc_abstract_filter):
    """
    remove_outliers - Remove outlier points by applying a statistical method on every point.
        See https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html
        See https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html
        Arguments:
            kNeighbours : How many neighbour points to take into account (int)
            threshold: threshold standard deviation multiplier (float)
            perTile: If true run the algorithm per tile (default: over the whole pointcloud)
    """
    filtername = "remove_outliers"
    
    def __init__(self, kNeighbours : int, threshold : float, perTile : bool=False):
        self.kNeighbours = kNeighbours
        self.threshold = threshold
        self.perTile = perTile
        self.count = 0
        self.times = []
        self.original_pointcounts = []
        self.pointcounts = []
        
    def filter(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        self.count += 1
        t1_d = time.time()
        self.original_pointcounts.append(pc.count())
        t1_o = time.time()
        clean_pc = cwipc_remove_outliers(pc, self.kNeighbours, self.threshold, self.perTile)
        pc.free()
        pc = clean_pc
        t2_o = time.time()
        self.times.append(t2_o-t1_o)
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

CustomFilter = RemoveOutliersFilter
