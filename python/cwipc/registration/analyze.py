
from typing import List, Optional, Any, Tuple
try:
    from typing import override
except ImportError:
    from typing_extensions import override
import math
import copy
import numpy as np
from numpy.typing import NDArray
import scipy.spatial
import scipy.stats
import open3d
from .. import cwipc_wrapper, cwipc_tilefilter
from .abstract import *
from .util import get_tiles_used, BaseAlgorithm, algdoc

KD_TREE_TYPE = scipy.spatial.KDTree

class BaseRegistrationAnalyzer(AnalysisAlgorithm, BaseAlgorithm):
    """
    Analyzes how good pointclouds are registered.

    Create the registrator, add pointclouds, run the algorithm, inspect the results.
    This is the base class, see the subclasses for various different algorithms.
    """
    #: Internal variables, may be useful for inspection
    source_ndarray : Optional[NDArray[Any]]
    target_ndarray : Optional[NDArray[Any]]
    target_kdtree : Optional[KD_TREE_TYPE]
    # Internal variable: the results
    results : AnalysisResults

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.histogram_bincount = 400
        self.max_correspondence_distance : float = np.inf
        self.histogram_binsize : float = 0.0
        self.correspondence_method : Optional[str] = None
        self.source_ndarray = None
        self.reference_ndarray = None
        self.reference_kdtree = None
        self.results = AnalysisResults()
        self.gaussian_bw_method = None
        self.ignore_nearest = 0

    @override
    def set_correspondence_method(self, method : Optional[str]):
        self.correspondence_method = method

    @override
    def set_min_correspondence_distance(self, correspondence: float) -> None:
        """Set the minimum correspondence distance that is meaningful"""
        self.histogram_binsize = correspondence

    @override
    def set_max_correspondence_distance(self, correspondence: float) -> None:
        """Set the maximum correspondence distance (above this points are not matched)"""
        self.max_correspondence_distance = correspondence

    @override
    def set_ignore_nearest(self, ignore_nearest: int) -> None:
        """Set the number of nearest points to ignore"""
        self.ignore_nearest = ignore_nearest

    def _prepare(self):
        self.source_ndarray = None
        self.reference_ndarray = None
        self.reference_kdtree = None
        self._prepare_results()
        self._prepare_source_ndarray()
        self._prepare_reference_ndarray()
        self._prepare_reference_kdtree()

    def _prepare_results(self):
        self.results = AnalysisResults()
        self.results.tilemask = self.source_tilemask
        self.results.referenceTilemask = self.reference_tilemask

    def _prepare_source_ndarray(self):
        assert self.source_pointcloud
        self.source_ndarray = self.source_pointcloud.get_numpy_matrix(onlyGeometry=True)
        self.results.sourcePointCount = self.source_pointcloud.count()

    def _prepare_reference_ndarray(self):
        assert self.reference_pointcloud
        self.reference_ndarray = self.reference_pointcloud.get_numpy_matrix(onlyGeometry=True)
        self.results.referencePointCount = self.reference_pointcloud.count()

    def _prepare_reference_kdtree(self):
        assert self.reference_ndarray is not None
        self.reference_kdtree = KD_TREE_TYPE(self.reference_ndarray)

    def _kdtree_get_distances_for_points(self, tree : KD_TREE_TYPE, points : NDArray[Any]) -> NDArray[Any]:
        """For each point in points, get the distance to the nearest point in the tree"""
        distances, _ = tree.query(points, k=[self.ignore_nearest+1], workers=-1, distance_upper_bound=self.max_correspondence_distance)
        return distances
    
    def _filter_infinites(self, distances : NDArray[Any]) -> NDArray[Any]:
        bitmap = np.isfinite(distances)
        return distances[bitmap]
    
    def _kdtree_count(self, tree : KD_TREE_TYPE) -> int:
        return tree.data.shape[0]

    @override
    def get_results(self) -> AnalysisResults:
        """Returns the analisys results.
        """
        assert self.results
        return self.results
    
    def _mode_from_histogram(self, histogram, histogramEdges) -> Tuple[float, float]:
        mode_index = np.argmax(histogram)
        mode = histogramEdges[mode_index+1]
        sigma = histogramEdges[mode_index+1] - histogramEdges[mode_index]
        return mode, sigma

    def _tail_from_histogram(self, histogram, histogramEdges) -> Tuple[float, float]:
        current_histogram = histogram
        current_histogramEdges = histogramEdges
        binFactor = 1
        while True:
            derivative = self._compute_derivative(current_histogram)
            if self._continuous_increase_decrease(derivative):
                # We have the correct binsize.
                return self._mode_from_histogram(current_histogram, current_histogramEdges)
            # We need to increase the binsize.
            if self.verbose:
                print(f"\t\tIncreasing binFactor from {binFactor} to {binFactor+1} to recompute histogram")
            binFactor = binFactor+1
            current_histogram, current_histogramEdges = self._recompute_histogram(histogram, histogramEdges, binFactor)
        
    def _compute_derivative(self, histogram: NDArray[Any]) -> NDArray[Any]:
        return np.diff(histogram, prepend=0)
    
    def _continuous_increase_decrease(self, derivative: NDArray[Any]) -> bool:
        length = derivative.shape[0]
        i = 0
        while i < length and derivative[i] >= 0:
            i += 1
        while i < length and derivative[i] <= 0:
            i += 1
        return i == length
    
    def _compute_histogram_parameters(self, distances: NDArray[Any]) -> bool:
        """Ensure histogram_binsize and histogram_bincount are set and consistent. Return False if all distances are the same."""
        max_distance = np.max(distances)
        min_distance = np.min(distances)
        if min_distance == max_distance:
            return False
        if min_distance > 0:
                min_distance = 0
        if self.histogram_binsize > 0:
            # We have a binsize (set by our caller), which is the minimum granularity we're interested in. Compute the binsize from this.
            self.histogram_bincount = int((max_distance - min_distance) / self.histogram_binsize)
            if self.verbose:
                print(f"\t\tmin={min_distance}, max={max_distance}, bincount={self.histogram_bincount} (based on min_correspondence_distance={self.histogram_binsize})")
        else:
            assert self.histogram_bincount > 0, "Either histogram_binsize or histogram_bincount must be set"
            self.histogram_binsize = (max_distance - min_distance) / self.histogram_bincount
            if self.verbose:
                print(f"\t\tmin={min_distance}, max={max_distance}, min_correspondence_distance={self.histogram_binsize} (based on bincount={self.histogram_bincount})")
        mismatch = (max_distance - min_distance) - (self.histogram_bincount * self.histogram_binsize)
        assert abs(mismatch) <= self.histogram_binsize, f"Mismatch in histogram parameters: mismatch={mismatch} (max={max_distance}, min={min_distance}, bincount={self.histogram_bincount}, binsize={self.histogram_binsize})"
        return True
                
    def _compute_histogram(self, raw_distances: NDArray[Any]) -> Tuple[NDArray[Any], NDArray[Any]]:
        assert self.histogram_bincount > 0
        assert self.histogram_binsize > 0
        return np.histogram(raw_distances, bins=self.histogram_bincount)
    
    def _compute_histogram_kde(self, raw_distances: NDArray[Any]) -> Tuple[NDArray[Any], NDArray[Any]]:
        assert self.histogram_bincount > 0
        assert self.histogram_binsize > 0
        kde = scipy.stats.gaussian_kde(raw_distances, bw_method=self.gaussian_bw_method)
        if self.verbose:
            print(f"\t\tgaussian_kde: bandwidth={kde.factor}, nPoint={len(raw_distances)}")
        edges = np.linspace(0, np.max(raw_distances), self.histogram_bincount + 1)
        values = kde.evaluate(edges[1:])
        return values, edges
        
    def _recompute_histogram(self, histogram: NDArray[Any], histogramEdges: NDArray[Any], binFactor: int) -> Tuple[NDArray[Any], NDArray[Any]]:
        assert binFactor > 0
        newBinCount = int(histogram.shape[0] / binFactor)
        newHistogram = np.zeros((newBinCount,), dtype=histogram.dtype)
        newHistogramEdges = np.zeros((newBinCount+1,), dtype=histogramEdges.dtype)
        newHistogramEdges[0] = histogramEdges[0]
        for i in range(newBinCount):
            newHistogramEdges[i+1] = histogramEdges[(i+1) * binFactor]
            for j in range(binFactor):
                if i * binFactor + j < histogram.shape[0]:
                    newHistogram[i] += histogram[i * binFactor + j]
        return newHistogram, newHistogramEdges

    def _compute_correspondence_errors(self, raw_distances: NDArray[Any]) -> None:
        overlap_distances = copy.deepcopy(raw_distances)
        mean = 0
        stddev = 0
        
        median = float(np.median(overlap_distances))
        mean = float(np.mean(overlap_distances))
        stddev = float(np.std(overlap_distances))
        if self.verbose:
            print(f"\t\tmedian={median}, mean={mean}, std={stddev}, nPoint={len(overlap_distances)}")
            
        # Last step: see how many points are below our new-found correspondence
        assert self.results
        if self.correspondence_method == None or self.correspondence_method == "mean":
            self.results.minCorrespondence = mean
            self.results.minCorrespondenceSigma = stddev
        elif self.correspondence_method == "median":
            self.results.minCorrespondence = median
            self.results.minCorrespondenceSigma = 0
        elif self.correspondence_method == "mode":
            tail, tailSigma = self._mode_from_histogram(self.results.histogram, self.results.histogramEdges)
            self.results.minCorrespondence = tail
            self.results.minCorrespondenceSigma = tailSigma
            if self.verbose:
                print(f"\t\tmode={tail}, modeSigma={tailSigma}")
        elif self.correspondence_method == "tail":
            tail, tailSigma = self._tail_from_histogram(self.results.histogram, self.results.histogramEdges)
            self.results.minCorrespondence = tail
            self.results.minCorrespondenceSigma = tailSigma
            if self.verbose:
                print(f"\t\ttail={tail}, tailSigma={tailSigma}")
        else:
            assert False, f"Unknown correspondence_method '{self.correspondence_method}'"
        filter = raw_distances <= self.results.minCorrespondence + self.results.minCorrespondenceSigma
        matched_point_count = np.count_nonzero(filter)
        self.results.minCorrespondenceCount = matched_point_count
        total_point_count = self.results.sourcePointCount
        fraction = matched_point_count/total_point_count
        if self.verbose:
            print(f"\t\tresult: tilemask={self.results.tilemask}, corr={self.results.minCorrespondence}, sigma={self.results.minCorrespondenceSigma}, nPoint={matched_point_count} of {total_point_count}, fraction={fraction}")



class RegistrationAnalyzer(BaseRegistrationAnalyzer):
    """
    This algorithm computes the registration between two point clouds.
    It uses scipy.spatial.KDtree.query to find the distances between points in the two clouds.
    """
    
    def __init__(self):
        BaseRegistrationAnalyzer.__init__(self)

    @override
    def run(self) -> bool:
        """Run the algorithm"""
        self._prepare()
        assert self.source_ndarray is not None
        assert self.reference_kdtree is not None
        distances = self._kdtree_get_distances_for_points(self.reference_kdtree, self.source_ndarray)
        distances = self._filter_infinites(distances)
        
        if not self._compute_histogram_parameters(distances):
            print("Warning: all distances are the same")
            value = distances[0]
            self.results.minCorrespondence = value
            self.results.minCorrespondenceSigma = 0
            self.results.minCorrespondenceCount = distances.shape[0]
            self.results.histogram = [value]
            self.results.histogramEdges = [value, value]
            return False
        histogram, edges = self._compute_histogram_kde(distances)
        self.results.histogram = histogram
        self.results.histogramEdges = edges
        self._compute_correspondence_errors(distances)
        return True


class OverlapAnalyzer(OverlapAnalysisAlgorithm, BaseAlgorithm):
    """
    This algorithm computes the overlap between two point clouds.
    It uses open3d.pipelines.registration.evaluate_registration to compute this number.
    """
    results : Optional[OverlapAnalysisResults]

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.correspondence : float = np.inf 
        self.results = None

    def set_correspondence(self, correspondence: float) -> None:
        """Set the correspondence distance"""
        self.correspondence = correspondence

    def _get_source_o3d_pointcloud(self) -> open3d.geometry.PointCloud:
        assert self.source_pointcloud
        source_o3d_pointcloud = open3d.geometry.PointCloud()
        source_points_nparray = self.source_pointcloud.get_numpy_matrix(onlyGeometry=True)
        source_o3d_pointcloud.points = open3d.utility.Vector3dVector(source_points_nparray)
        return source_o3d_pointcloud
    
    def _get_reference_o3d_pointcloud(self) -> open3d.geometry.PointCloud:
        assert self.reference_pointcloud
        reference_o3d_pointcloud = open3d.geometry.PointCloud()
        reference_points_nparray = self.reference_pointcloud.get_numpy_matrix(onlyGeometry=True)
        reference_o3d_pointcloud.points = open3d.utility.Vector3dVector(reference_points_nparray)
        return reference_o3d_pointcloud
    
    @override
    def run(self) -> bool:
        """Run the algorithm"""
        source_o3d_pointcloud = self._get_source_o3d_pointcloud()
        reference_o3d_pointcloud = self._get_reference_o3d_pointcloud()
        result = open3d.pipelines.registration.evaluate_registration(
            source_o3d_pointcloud, reference_o3d_pointcloud, max_correspondence_distance=self.correspondence)
        self.results = OverlapAnalysisResults()
        self.results.fitness = result.fitness
        self.results.rmse = result.inlier_rmse
        self.results.sourcePointCount = len(source_o3d_pointcloud.points)
        self.results.referencePointCount = len(reference_o3d_pointcloud.points)
        self.results.tilemask = self.source_tilemask
        self.results.referenceTilemask = self.reference_tilemask
        return True

    @override
    def get_results(self) -> OverlapAnalysisResults:
        """Returns the analisys results.
        """
        assert self.results
        return self.results

DEFAULT_ANALYZER_ALGORITHM = RegistrationAnalyzer

ALL_ANALYZER_ALGORITHMS = [
    RegistrationAnalyzer,
    OverlapAnalyzer
]

HELP_ANALYZER_ALGORITHMS = """
## Analyzer algorithms

The analyzer algorithm looks at a source point cloud and a target point cloud and tries to determine how well they are aligned.

The default analyzer algorithm is """ + DEFAULT_ANALYZER_ALGORITHM.__name__ + """

The following analyzer algorithms are available:
""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_ANALYZER_ALGORITHMS])