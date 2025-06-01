
from typing import List, Optional, Any, Tuple, override
import math
import copy
import numpy as np
from numpy.typing import NDArray
import scipy.spatial
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
    #: Minimum distance between points (we will not search for nearer points once we found one within eps)
    eps : float
    #: Maximum distance between points that could potentially match.
    distance_upper_bound : float
    #: Internal variables, may be useful for inspection
    source_ndarray : Optional[NDArray[Any]]
    target_ndarray : Optional[NDArray[Any]]
    target_kdtree : Optional[KD_TREE_TYPE]
    # Internal variable: the results
    results : AnalysisResults

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.histogram_bincount = 400
        self.eps = 0.0
        self.distance_upper_bound = 0.2
        self.source_ndarray = None
        self.reference_ndarray = None
        self.reference_kdtree = None
        self.results = AnalysisResults()

    @override
    def set_source_pointcloud(self, pc: cwipc_wrapper, tilemask: Optional[int] = None) -> None:
        """Set the source point cloud for this algorithm"""
        super().set_source_pointcloud(pc, tilemask)
        
    @override
    def set_reference_pointcloud(self, pc: cwipc_wrapper, tilemask: Optional[int] = None) -> None:
        """Set the reference point cloud for this algorithm"""
        super().set_reference_pointcloud(pc, tilemask)

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

    @override
    def run(self) -> bool:
        assert False

    def _kdtree_get_distances_for_points(self, tree : KD_TREE_TYPE, points : NDArray[Any]) -> NDArray[Any]:
        """For each point in points, get the distance to the nearest point in the tree"""
        distances, _ = tree.query(points, workers=-1)
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
    
    def _compute_correspondence_errors(self, raw_distances: NDArray[Any]) -> None:
        overlap_distances = copy.deepcopy(raw_distances)
        mean = 0
        stddev = 0
        
        mean = float(np.mean(overlap_distances))
        stddev = float(np.std(overlap_distances))
        if self.verbose:
            print(f"\t\tmean={mean}, std={stddev}, nPoint={len(overlap_distances)}")
            
        # Last step: see how many points are below our new-found correspondence
        filter = raw_distances <= mean
        matched_point_count = np.count_nonzero(filter)
        assert self.results
        self.results.minCorrespondence = mean
        self.results.minCorrespondenceSigma = stddev
        self.results.minCorrespondenceCount = matched_point_count
        total_point_count = self.results.sourcePointCount
        fraction = matched_point_count/total_point_count
        if self.verbose:
            print(f"\t\tresult: tilemask={self.results.tilemask}, corr={mean}, sigma={stddev}, nPoint={matched_point_count} of {total_point_count}, fraction={fraction}")

    @override
    def filter_sources(self) -> None:
        """Filter points that were matched by a previous run"""
        assert False
        nCamera = len(self.per_camera_pointclouds)
        assert self.results
        for cam_i in range(nCamera):
            assert len(self.per_camera_nparray[cam_i]) > 0
            assert len(self.per_camera_nparray_distances[cam_i]) > 0
            assert self.results.minCorrespondence[cam_i] > 0
            assert self.results.minCorrespondenceSigma[cam_i] > 0
            cutoff = self.results.minCorrespondence[cam_i] + self.results.minCorrespondenceSigma[cam_i] # xxxjack copilot suggested 2*sigma...
            old_count = len(self.per_camera_nparray[cam_i])
            filter = self.per_camera_nparray_distances[cam_i] > cutoff
            filtered_nparray = self.per_camera_nparray[cam_i][filter]
            self.per_camera_nparray[cam_i] = filtered_nparray
            self.per_camera_nparray_distances[cam_i] = self.per_camera_nparray_distances[cam_i][filter]            
            new_count = len(filtered_nparray)
            if self.verbose:
                print(f"filter_sources: camera {cam_i}: from {old_count} to {new_count} points, distance={cutoff}")
        self.filter_label = f" (filtered)"
        self.per_camera_kdtree = []
        self.per_camera_histograms = []

class RegistrationAnalyzer(BaseRegistrationAnalyzer):
    """
    This algorithm computes the registration between two point clouds.
    """
    
    def __init__(self):
        BaseRegistrationAnalyzer.__init__(self)

    def run(self) -> bool:
        """Run the algorithm"""
        self._prepare()
        assert self.source_ndarray is not None
        assert self.reference_kdtree is not None
        distances = self._kdtree_get_distances_for_points(self.reference_kdtree, self.source_ndarray)
        distances = self._filter_infinites(distances)
        self._compute_correspondence_errors(distances)
        histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
        self.results.histogram = histogram
        self.results.histogramEdges = edges
        return True

class RegistrationAnalyzerIgnoreNearest(RegistrationAnalyzer):
    """
    This algorithm computes the registration between two point clouds, ignoring the nearest point.
    This is useful to match a point cloud to itself.
    """
    
    def _kdtree_get_distances_for_points(self, tree : KD_TREE_TYPE, points : NDArray[Any]) -> NDArray[Any]:
        """For each point in points, get the distance to the nearest point in the tree"""
        distances, _ = tree.query(points, k=[2], workers=-1)
        return distances
    
## xxxjack we need a second-order registration analyzer, which computes the second-best correspondence
      
DEFAULT_ANALYZER_ALGORITHM = RegistrationAnalyzer

ALL_ANALYZER_ALGORITHMS = [
    RegistrationAnalyzer,
    RegistrationAnalyzerIgnoreNearest
]

HELP_ANALYZER_ALGORITHMS = """
The analyzer algorithm looks at a source point cloud and a target point cloud and tries to determine how well they are aligned.

The following multicamera analyzer algorithms are available:
""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_ANALYZER_ALGORITHMS])