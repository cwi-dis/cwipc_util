
from typing import List, Optional, Any, Tuple, cast, Container, Union, Iterable
import math
import copy
import numpy as np
from numpy.typing import NDArray
import scipy.spatial
from matplotlib import pyplot as plt
from .. import cwipc_wrapper, cwipc_tilefilter
from .abstract import *
from .util import get_tiles_used, BaseAlgorithm, algdoc

KD_TREE_TYPE = scipy.spatial.KDTree
PLOT_COLORS = ["r", "g", "b", "y", "m", "c", "orange", "lime"] # 8 colors. First 4 match cwipc_tilecolor().

DEFAULT_PLOT_STYLE = ["count"]

def set_default_plot_style(style : Union[str, Iterable[str]]):
    global DEFAULT_PLOT_STYLE
    if isinstance(style, str):
        DEFAULT_PLOT_STYLE = ','.split(style)
    else:
        DEFAULT_PLOT_STYLE = list(style)
        
class BaseRegistrationAnalyzer(AnalysisAlgorithm, BaseAlgorithm):
    """
    Analyzes how good pointclouds are registered.

    Create the registrator, add pointclouds, run the algorithm, inspect the results.
    Attributes histogram_bincount and label can be changed before running.

    This is the base class, see the subclasses for various different algorithms.
    """

    #: Number of bins we want in the histogram
    histogram_bincount : int
    #: Minimum distance between points (we will not search for nearer points once we found one within eps)
    eps : float
    #: Maximum distance between points that could potentially match.
    distance_upper_bound : float
    #: Label for the plot
    plot_label : Optional[str] 
    #: Title for the plot (usually determined by this class)
    plot_title : str
    #: Internal variable, may be useful for inspection: numpy array of all points in this tile
    per_camera_nparray : List[NDArray[Any]]
    #: Internal variable, may be useful for inspection: numpy array of all points in all other tiles
    per_camera_nparray_others : List[NDArray[Any]]
    #: Internal variable, may be useful for inspection: kdtree of this tile
    per_camera_kdtree : List[KD_TREE_TYPE]
    #: Internal variable, may be useful for inspection: kdtree of all other tiles 
    per_camera_kdtree_others : List[KD_TREE_TYPE]
    #: Internal variable, may be useful for inspection: histogram results. List of tuples with histogram-data, edges, histogram-cumulative, histogram-cumulative-normalized, plot_label, raw-distance-data
    per_camera_histograms : List[Optional[Tuple[NDArray[Any], NDArray[Any], NDArray[Any], NDArray[Any], str, NDArray[Any]]]]
    # Internal variable: (per-camera) computed correspondence error
    correspondence_errors : List[float]
    # Internal variable: (per-camera, or pair-wise) count of points that are considered to be mappable to the other cloud
    matched_point_counts : List[int]
    # Internal variable: (per-camera, or pair-wise) fraction of all points that are considered to be mappable
    matched_point_fractions : List[float]
    
    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.histogram_bincount = 400
        self.eps = 0.0
        self.distance_upper_bound = 0.2
        self.plot_label = None
        self.plot_title = "Unknown RegistrationAnalyzer"
        self.per_camera_nparray = []
        self.per_camera_nparray_others = []
        self.per_camera_kdtree  = []
        self.per_camera_kdtree_others = [] 
        self.per_camera_histograms = []
        self.correspondence_errors = []
        self.matched_point_counts  = []
        self.matched_point_fractions = []

    def run(self, target: Optional[int]=None) -> bool:
        assert False


    def _kdtree_get_distances_to_points(self, tree : KD_TREE_TYPE, points : NDArray[Any]) -> NDArray[Any]:
        distances, _ = tree.query(points, workers=-1)
        bitmap = np.isfinite(distances)
        filtered_distances = distances[bitmap]
        return filtered_distances
    
    def _kdtree_count(self, tree : KD_TREE_TYPE) -> int:
        return tree.data.shape[0]

    def plot(self, filename : Optional[str]=None, show : bool = False, which : Optional[Container[str]]=None):
        # xxxjack This uses the stateful pyplot API. Horrible.
        if not filename and not show:
            return
        if which is None:
            which = DEFAULT_PLOT_STYLE
        do_count = which is None or 'count' in which or 'all' in which
        do_cumulative = which is None or 'cumulative' in which or 'all' in which
        do_delta = which is None or 'delta' in which or 'all' in which
        do_log = which is not None and 'log' in which
        
        nCamera = len(self.per_camera_histograms)
        plot_fig, plot_ax = plt.subplots()
        if do_log:
            plot_ax.set_yscale('log')
        ax_cum = None
        if do_cumulative:
            ax_cum = plot_ax.twinx()
            if do_log:
                ax_cum.set_yscale('log')
        corr_box_text = "Correspondence error:\n"
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            h_data = self.per_camera_histograms[cam_i]
            corr = self.correspondence_errors[cam_i]
            count = self.matched_point_counts[cam_i]
            percentage = int(self.matched_point_fractions[cam_i] * 100)
            corr_box_text += f"\n{cam_tilenum}: {corr:.4f} ({count} points, {percentage}%)"
            assert h_data
            (histogram, edges, cumsum, normsum, plot_label, raw_distances) = h_data
            plot_ax.plot(edges[1:], histogram, label=plot_label, color=PLOT_COLORS[cam_i])
            if do_cumulative:
                assert ax_cum
                ax_cum.plot(edges[1:], normsum, linestyle="dashed", label="_nolegend_", color=PLOT_COLORS[cam_i])
                ax_cum.plot([corr, corr], [0, 1], linestyle="dotted", label="_nolegend_", color=PLOT_COLORS[cam_i])
            
        title = self.plot_title
        if self.plot_label:
            title = self.plot_label + "\n" + title
        plt.title(title)
        props = dict(boxstyle='round', facecolor='white', alpha=0.5)
        plot_ax.text(0.98, 0.1, corr_box_text, transform=plot_ax.transAxes, fontsize='small', verticalalignment='bottom', horizontalalignment="right", bbox=props)
        plot_ax.legend()
        if filename:
            plt.savefig(filename)
        if show:
            plt.show()
   
    def get_ordered_results(self, weightstyle : str = 'priority') -> List[Tuple[int, float, float]]:
        """Returns a list of tuples (cameraNumber, correspondenceError, weight), ordered by weight (highest first)
        
        This is the order in which the camera re-registration should be attempted.
        """
        rv = []
        for camnum in range(len(self.correspondence_errors)):
            if weightstyle == 'priority':
                # Option 1: Use the correspondence as-is
                #weight = self.correspondences[camnum]
                # Option 2: multiply by the number of points that were matched
                #weight = self.correspondences[camnum]*self.below_correspondence_counts[camnum]
                # option 3: multiply by the square root of the number of matched points
                #weight = self.correspondences[camnum]*math.sqrt(self.below_correspondence_counts[camnum])
                # option 4: multiply by the log of the number of matched points
                weight = math.log(self.matched_point_counts[camnum]) * self.correspondence_errors[camnum]
            elif weightstyle == 'match':
                weight = math.log(self.matched_point_counts[camnum]) / self.correspondence_errors[camnum]
            else:
                assert False, f"get_ordered_results: unknown weightstyle {weightstyle}"
            rv.append((self.per_camera_tilenum[camnum], self.correspondence_errors[camnum], weight))
        rv.sort(key=lambda t:-t[2])
        return rv
    
    def _prepare(self):
        assert False

    def _prepare_nparrays(self):
        assert self.per_camera_nparray == []
        self.per_camera_nparray = [
            cam_pc.get_numpy_matrix(onlyGeometry=True) for cam_pc in self.per_camera_pointclouds
        ]
    
    def _prepare_kdtrees(self):
        assert self.per_camera_kdtree == []
        self.per_camera_kdtree = [ # type: ignore
            KD_TREE_TYPE(points) for points in self.per_camera_nparray
        ]

    def _prepare_nparrays_others(self):
        assert self.per_camera_nparray_others == []
        for camnum in range(len(self.per_camera_nparray)):
            # Create an array of all points except the ones for camnum
            tmp = copy.copy(self.per_camera_nparray)
            del tmp[camnum]
            other_points = np.concatenate(tmp)
            self.per_camera_nparray_others.append(other_points)

    def _prepare_kdtrees_others(self):
        assert self.per_camera_kdtree_others == []
        if self.per_camera_nparray_others == []:
            self._prepare_nparrays_others()
        self.per_camera_kdtree_others = [ # type: ignore
            KD_TREE_TYPE(points) for points in self.per_camera_nparray_others
        ]

    def _compute_correspondence_errors(self):
        N_FILTERING_STEPS = 2
        nCamera = len(self.per_camera_histograms)
        assert self.correspondence_errors == []
        assert self.matched_point_counts == []
        assert self.matched_point_fractions == []
        if self.verbose:
            print(f"{self.__class__.__name__}: computing correspondence errors:")
        for cam_i in range(nCamera):
            tilenum = self.per_camera_tilenum[cam_i]
            if self.verbose:
                print(f"\tcamera {tilenum}:")
            hdata = self.per_camera_histograms[cam_i]
            assert hdata
            histogram, edges, cumsum, normsum, plot_label, raw_distances = hdata
            overlap_distances = copy.deepcopy(raw_distances)
            mean = 0
            stddev = 0
            for filterstep in range(N_FILTERING_STEPS):
                mean = float(np.mean(overlap_distances))
                stddev = float(np.std(overlap_distances))
                if self.verbose:
                    print(f"\t\tstep {filterstep}: mean={mean}, std={stddev}, nPoint={len(overlap_distances)}")
                # Create an array of booleans for all distances we want to keep, and filter on that.
                # filter = np.logical_and(overlap_distances <= (mean+stddev), overlap_distances >= (mean-stddev))
                filter = np.logical_and(overlap_distances <= (mean+stddev), overlap_distances >= 0)
                overlap_distances = overlap_distances[filter]
            # Last step: see how many points are below our new-found correspondence
            corr = mean + stddev
            filter = raw_distances <= corr
            matched_point_count = np.count_nonzero(filter)
            self.correspondence_errors.append(corr)
            self.matched_point_counts.append(matched_point_count)
            total_point_count = len(raw_distances)
            fraction = matched_point_count/total_point_count
            if self.verbose:
                print(f"\t\tresult: mean+std=corr={corr}, nPoint={matched_point_count} of {total_point_count}, fraction={fraction}")
            self.matched_point_fractions.append(fraction)

class RegistrationAnalyzerNoOp(BaseRegistrationAnalyzer):
    """
    This algorithm does nothing, it just returns a single tile with a single point.
    """
    def run(self, target: Optional[int]=None) -> bool:
        assert target is None
        assert len(self.per_camera_pointclouds) > 0
        self.per_camera_histograms = [
            (np.array([1]), np.array([0,1]), np.array([1]), np.array([1]), f"single tile {self.per_camera_tilenum[0]}", np.array([]))
        ]
        self.correspondence_errors = [0.0]
        self.matched_point_counts = [1]
        return True
    
class RegistrationPairFinder(BaseRegistrationAnalyzer):
    """
    This algorithm computes, for each pair of tiles, the number of overlapping points and the distribution of their point2point distances.
    """

    def __init__(self):
        BaseRegistrationAnalyzer.__init__(self)
        self.plot_title = "Histogram of point distances between camera pairs"


    def _prepare(self):
        self._prepare_nparrays()
        self._prepare_kdtrees()

    def run(self, target: Optional[int]=None) -> bool:
        """Run the algorithm"""
        assert target is None
        assert len(self.per_camera_pointclouds) > 0
        if len(self.per_camera_pointclouds) == 1:
            # If there is only a single tile we have nothing to do.
            self.per_camera_histograms = [
                (np.array([1]), np.array([0,1]), np.array([1]), np.array([1]), f"single tile {self.per_camera_tilenum[0]}", np.array([]))
            ]
            self.correspondence_errors : List[float] = [0.0]
            self.below_correspondence_error_counts : List[int] = [1]
            return True
        self._prepare()
        # Ensure we have the arrays and kdtrees we need
        assert self.per_camera_nparray
        assert self.per_camera_kdtree

        nCamera = len(self.per_camera_pointclouds)
        nPairs = (nCamera * (nCamera-1)) // 2   # That is not a comment, that is an integer divide:-)
        combined_tilenum : List[int] = [0] * nPairs
        self.per_camera_histograms = [None]*nPairs
        idx = 0
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            src_points = self.per_camera_nparray[cam_i]
            src_kdtree = self.per_camera_kdtree[cam_i]
            for cam_j in range(cam_i+1, nCamera):
                dst_tilenum = self.per_camera_tilenum[cam_j]
                new_tilenum = cam_tilenum | dst_tilenum
                dst_kdtree = self.per_camera_kdtree[cam_j]
                dst_points = self.per_camera_nparray[cam_j]
                # Compute symmetric distances
                distances_1 = self._kdtree_get_distances_to_points(dst_kdtree, src_points)
                distances_2 = self._kdtree_get_distances_to_points(src_kdtree, dst_points)
                # Now remove distances from the largest pointcloud (because they are bullshit)
                count_1 = distances_1.shape[0]
                count_2 = distances_2.shape[0]
                shortest_count = min(count_1, count_2)
                distances_1 = distances_1[:shortest_count]
                distances_2 = distances_2[:shortest_count]
                distances = np.concatenate((distances_1, distances_2))
                histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
                cumsum = np.cumsum(histogram)
                totPoints = src_points.shape[0]
                totOtherPoints = dst_points.shape[0]
                totDistances = distances.shape[0]
                normsum = cumsum / totDistances
                plot_label = f"{new_tilenum} ({totPoints} points to {totOtherPoints})"
                self.per_camera_histograms[idx] = (histogram, edges, cumsum, normsum, plot_label, distances)
                combined_tilenum[idx] = new_tilenum
                idx += 1
        self.per_camera_tilenum = combined_tilenum
        self._compute_correspondence_errors()
        return True

class RegistrationAnalyzer(BaseRegistrationAnalyzer):
    """
    This algorithm computes, for each tile, the number of overlapping points with any of the other tiles and the distribution of their point2point distances.
    """
    
    def __init__(self):
        BaseRegistrationAnalyzer.__init__(self)
        self.plot_title = "Histogram of point distances between camera and all others"

    def _prepare(self):
        self._prepare_nparrays()
        self._prepare_kdtrees_others()

    def run(self, target: Optional[int]=None) -> bool:
        """Run the algorithm"""
        assert target is None
        assert len(self.per_camera_pointclouds) > 0
        if len(self.per_camera_pointclouds) == 1:
            # If there is only a single tile we have nothing to do.
            self.per_camera_histograms = [
                (np.array([1]), np.array([0,1]), np.array([1]), np.array([1]), f"single tile {self.per_camera_tilenum[0]}", np.array([]))
            ]
            self.correspondence_errors = [0.0]
            self.matched_point_counts = [1]
            return True
        self._prepare()
        # Ensure we have the arrays and kdtrees we need
        assert self.per_camera_nparray
        assert self.per_camera_kdtree_others

        nCamera = len(self.per_camera_pointclouds)
        self.per_camera_histograms  = [None] * nCamera
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            src_points = self.per_camera_nparray[cam_i]
            dst_kdtree = self.per_camera_kdtree_others[cam_i]
            distances = self._kdtree_get_distances_to_points(dst_kdtree, src_points)
            histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
            cumsum = np.cumsum(histogram)
            totPoints = cumsum[-1]
            totOtherPoints = self._kdtree_count(dst_kdtree)
            normsum = cumsum / totPoints
            plot_label = f"{cam_tilenum} ({totPoints} points to {totOtherPoints})"
            self.per_camera_histograms[cam_i] = (histogram, edges, cumsum, normsum, plot_label, distances)
        self._compute_correspondence_errors()
        return True

class RegistrationAnalyzerFiltered(RegistrationAnalyzer):
    """
    Same algorithm as RegistrationAnalyzer, but with an additional step to filter out points that are unlikely to have a match.
    
    All point clouds are first filtered to remove the points that are unlikely to have a match (no make the pointclouds and the kdtrees smaller)
    """
    
    def _prepare(self):
        self._prepare_nparrays()
        self._prepare_kdtrees()
        self._prepare_kdtrees_others()
        self._filter_nparrays()
        # Now the nparrays have been filtered. Recompute the kdtrees.
        self.per_camera_kdtree = []
        self.per_camera_kdtree_others = []
        self._prepare_kdtrees()
        self._prepare_kdtrees_others()

    def _filter_nparrays(self):
        """Filter each per_camera_nparray so it contains only points that have a chance of being close"""
        assert self.per_camera_nparray
        assert self.per_camera_kdtree_others
        for camnum in range(len(self.per_camera_nparray)):
            orig_nparray = self.per_camera_nparray[camnum]
            tree = self.per_camera_kdtree_others[camnum]
            distances, _ = tree.query(orig_nparray, eps=self.eps, distance_upper_bound=self.distance_upper_bound)
            bitmap = np.isfinite(distances)
            filtered_nparray = orig_nparray[bitmap]
            self.per_camera_nparray[camnum] = filtered_nparray
            if self.verbose:
                print(f"filter_nparrays: camera {camnum}: from {orig_nparray.shape[0]} to {filtered_nparray.shape[0]} points, distance={self.distance_upper_bound}")
                    

class RegistrationAnalyzerReverse(RegistrationAnalyzer):
    """
    Same algorithm as RegistrationAnalyzer, but reversed: the distances are computed for the other pointclouds to this pointcloud.
    """

    def __init__(self):
        RegistrationAnalyzer.__init__(self)
        self.plot_title = "Histogram of point distances between all others and camera"
        
    def _prepare(self):
        self._prepare_nparrays()
        self._prepare_nparrays_others()
        self._prepare_kdtrees()

    def run(self, target: Optional[int]=None) -> bool:
        """Run the algorithm"""
        assert target is None
        assert len(self.per_camera_pointclouds) > 0
        if len(self.per_camera_pointclouds) == 1:
            # If there is only a single tile we have nothing to do.
            self.per_camera_histograms = [
                (np.array([1]), np.array([0,1]), np.array([1]), np.array([1]), f"single tile {self.per_camera_tilenum[0]}", np.array([]))
            ]
            self.correspondence_errors = [0.0]
            self.matched_point_counts = [1]
            return True
        self._prepare()
        # Ensure we have the arrays and kdtrees we need
        assert self.per_camera_nparray_others
        assert self.per_camera_kdtree

        nCamera = len(self.per_camera_pointclouds)
        self.per_camera_histograms = [None] * nCamera
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            src_points = self.per_camera_nparray_others[cam_i]
            dst_kdtree = self.per_camera_kdtree[cam_i]
            distances = self._kdtree_get_distances_to_points(dst_kdtree, src_points)
            histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
            cumsum = np.cumsum(histogram)
            totPoints = cumsum[-1]
            totOtherPoints = self._kdtree_count(dst_kdtree)
            normsum = cumsum / totPoints
            plot_label = f"{cam_tilenum} ({totPoints} points to {totOtherPoints})"
            self.per_camera_histograms[cam_i] = (histogram, edges, cumsum, normsum, plot_label, distances)
        self._compute_correspondence_errors()
        return True
    
class RegistrationAnalyzerFilteredReverse(RegistrationAnalyzerReverse):
    """
    Same algorithm as RegistrationAnalyzerReverse, but with an additional step to filter out points that are unlikely to have a match.
    """
    
    def _prepare(self):
        self._prepare_nparrays()
        self._prepare_kdtrees()
        self._prepare_nparrays_others()
        self._filter_nparrays_others()
        
    def _filter_nparrays_others(self):
        """Filter each per_camera_nparray so it contains only points that have a chance of being close"""
        assert self.per_camera_nparray_others
        assert self.per_camera_kdtree
        for camnum in range(len(self.per_camera_nparray_others)):
            orig_nparray = self.per_camera_nparray_others[camnum]
            tree = self.per_camera_kdtree[camnum]
            distances, _ = tree.query(orig_nparray, eps=self.eps, distance_upper_bound=self.distance_upper_bound)
            bitmap = np.isfinite(distances)
            filtered_nparray = orig_nparray[bitmap]
            self.per_camera_nparray_others[camnum] = filtered_nparray
                    
DEFAULT_ANALYZER_ALGORITHM = RegistrationAnalyzer

ALL_ANALYZER_ALGORITHMS = [
    RegistrationAnalyzer,
    RegistrationAnalyzerFiltered,
    RegistrationAnalyzerReverse,
    RegistrationAnalyzerFilteredReverse,
    RegistrationPairFinder,
    RegistrationAnalyzerNoOp
]

HELP_ANALYZER_ALGORITHMS = """
The analyzer algorithm looks at a source point cloud and a target point cloud and tries to determine how well they are aligned.

The following analyzer algorithms are available:
""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_ANALYZER_ALGORITHMS])