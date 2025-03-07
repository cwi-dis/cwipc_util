
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
        DEFAULT_PLOT_STYLE = style.split(',')
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
    #: Internal variable, may be useful for inspection: numpy array of distances for all points in this tile
    per_camera_nparray_distances : List[NDArray[Any]]
    #: Internal variable, may be useful for inspection: numpy array of all points in all other tiles
    per_camera_nparray_others : List[NDArray[Any]]
    #: Internal variable, may be useful for inspection: kdtree of this tile
    per_camera_kdtree : List[KD_TREE_TYPE]
    #: Internal variable, may be useful for inspection: kdtree of all other tiles 
    per_camera_kdtree_others : List[KD_TREE_TYPE]
    #: Internal variable, may be useful for inspection: histogram results. List of tuples with histogram-data, edges, histogram-cumulative, histogram-cumulative-normalized, plot_label, raw-distance-data
    per_camera_histograms : List[Optional[Tuple[NDArray[Any], NDArray[Any], NDArray[Any], NDArray[Any], str, NDArray[Any]]]]
    # Internal variable: (per-camera, or pair-wise) fraction of all points that are considered to be mappable
    matched_point_fractions : List[float]
    # Internal variable: which pass number
    pass_number : int
    # Internal variable: the results
    results : Optional[AnalysisResults]

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.histogram_bincount = 400
        self.eps = 0.0
        self.distance_upper_bound = 0.2
        self.plot_label = None
        self.plot_title = "Unknown RegistrationAnalyzer"
        self.per_camera_nparray = []
        self.per_camera_nparray_distances = []
        self.per_camera_nparray_others = []
        self.per_camera_kdtree  = []
        self.per_camera_kdtree_others = [] 
        self.per_camera_histograms = []
        self.matched_point_fractions = []
        self.filter_label = ""
        self.pass_number = 0
        self.results = None

    def add_tiled_pointcloud(self, pc: cwipc_wrapper) -> None:
        super().add_tiled_pointcloud(pc)
        self.results = AnalysisResults(self.camera_count())
        self.results.tileNums = [i for i in self.per_camera_tilenum]
    
    def run(self, target: Optional[int]=None) -> bool:
        assert False

    def run_twice(self, target: Optional[int]=None) -> bool:
        ok = self.run(target)
        if not ok:
            return ok
        self.pass_number = 1
        self.filter_sources()
        return self.run(target)

    def _kdtree_get_distances_to_points(self, tree : KD_TREE_TYPE, points : NDArray[Any]) -> NDArray[Any]:
        distances, _ = tree.query(points, workers=-1)
        return distances
    
    def _filter_infinites(self, distances : NDArray[Any]) -> NDArray[Any]:
        bitmap = np.isfinite(distances)
        return distances[bitmap]
    
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
        do_log_cumulative = False # do_log
        nCamera = len(self.per_camera_histograms)
        plot_fig, plot_ax = plt.subplots()
        if do_log:
            plot_ax.set_yscale('symlog')
        plot_ax.set_xlabel("Distance (m)")
        plot_ax.set_ylabel(do_log and "log(count)" or "count")
        ax_cum = None
        if do_cumulative:
            ax_cum = plot_ax.twinx()
            if do_log_cumulative:
                ax_cum.set_yscale('log')
            ax_cum.set_ylabel(do_log_cumulative and "log(cumulative)" or "cumulative")
        corr_box_text = "Correspondence:\n"
        assert self.results
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            h_data = self.per_camera_histograms[cam_i]
            corr = self.results.minCorrespondence[cam_i]
            corr_sigma = self.results.minCorrespondenceSigma[cam_i]
            count = self.results.minCorrespondenceCount[cam_i]
            percentage = int(self.matched_point_fractions[cam_i] * 100)
            corr_box_text += f"\n{cam_tilenum}: {corr:.4f}±{corr_sigma:.4f} ({count} points, {percentage}%)"
            assert h_data
            (histogram, edges, cumsum, normsum, plot_label, raw_distances) = h_data
            plot_ax.plot(edges[1:], histogram, label=plot_label, color=PLOT_COLORS[cam_i])
            if do_cumulative:
                assert ax_cum
                ax_cum.plot(edges[1:], normsum, linestyle="dashed", label="_nolegend_", color=PLOT_COLORS[cam_i])
                ax_cum.plot([corr, corr], [0, 1], linestyle="dashed",  label="_nolegend_", color=PLOT_COLORS[cam_i])
            if do_delta:
                # Compute deltas over intervals of half of "corr" size
                
                corr_bin = int(np.digitize(corr, edges))
                nbin = len(histogram) // (corr_bin//2)
                while len(histogram) % nbin != 0:
                    nbin += 1
                new_edges = edges[0::nbin]
                new_histo = np.reshape(histogram, (-1, nbin)).sum(axis=1)/nbin
                delta = np.diff(new_histo)
                plot_ax.plot([new_edges[0], new_edges[-1]], [0, 0], linestyle="solid", label="_nolegend_", color="black", linewidth=0.2)
                plot_ax.plot(new_edges[1:-1], delta, marker=".", linewidth=0, label="_nolegend_", color=PLOT_COLORS[cam_i])
        title = self.plot_title
        if self.plot_label:
            title = self.plot_label + self.filter_label + "\n" + title
        plt.title(title)
        props = dict(boxstyle='round', facecolor='white', alpha=0.5)
        plot_ax.text(0.98, 0.1, corr_box_text, transform=plot_ax.transAxes, fontsize='small', verticalalignment='bottom', horizontalalignment="right", bbox=props)
        plot_ax.legend()
        if filename:
            plt.savefig(filename)
        if show:
            plt.show()
            plt.close()
   
    def get_results(self, weightstyle : str = 'priority') -> AnalysisResults:
        """Returns a list of tuples (cameraNumber, correspondenceError, weight), ordered by weight (highest first)
        
        This is the order in which the camera re-registration should be attempted.
        """
        assert self.results
        return self.results
    
    def _prepare(self):
        assert False

    def _prepare_nparrays(self):
        if self.per_camera_nparray == []:
            self.per_camera_nparray = [
                cam_pc.get_numpy_matrix(onlyGeometry=True) for cam_pc in self.per_camera_pointclouds
            ]
            self.per_camera_nparray_distances = [np.array([]) for _ in self.per_camera_pointclouds]
    
    def _prepare_kdtrees(self):
        if self.per_camera_kdtree == []:
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
        if self.per_camera_kdtree_others == []:
            if self.per_camera_nparray_others == []:
                self._prepare_nparrays_others()
            self.per_camera_kdtree_others = [ # type: ignore
                KD_TREE_TYPE(points) for points in self.per_camera_nparray_others
            ]

    def _compute_correspondence_errors(self):
        nCamera = len(self.per_camera_histograms)
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
            
            mean = float(np.mean(overlap_distances))
            stddev = float(np.std(overlap_distances))
            if self.verbose:
                print(f"\t\tmean={mean}, std={stddev}, nPoint={len(overlap_distances)}")
                
            # Last step: see how many points are below our new-found correspondence
            filter = raw_distances <= mean
            matched_point_count = np.count_nonzero(filter)
            assert self.results
            if self.pass_number == 0:
                self.results.minCorrespondence[cam_i] = mean
                self.results.minCorrespondenceSigma[cam_i] = stddev
                self.results.minCorrespondenceCount[cam_i] = matched_point_count
            else:
                self.results.secondCorrespondence[cam_i] = mean
                self.results.secondCorrespondenceSigma[cam_i] = stddev
                self.results.secondCorrespondenceCount[cam_i] = matched_point_count
            total_point_count = len(raw_distances)
            fraction = matched_point_count/total_point_count
            if self.verbose:
                print(f"\t\tresult: corr={mean}, sigma={stddev}, nPoint={matched_point_count} of {total_point_count}, fraction={fraction}")
            self.matched_point_fractions.append(fraction)

    def filter_sources(self) -> None:
        """Filter points that were matched by a previous run"""
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
        self.matched_point_fractions = []
        self.per_camera_histograms = []
    
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
                distances_1 = self._filter_infinites(distances_1)
                distances_2 = self._filter_infinites(distances_2)
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
                plot_label = f"{new_tilenum} ({totPoints} to {totOtherPoints})"
                self.per_camera_histograms[idx] = (histogram, edges, cumsum, normsum, plot_label, distances)
                combined_tilenum[idx] = new_tilenum
                idx += 1
        self.per_camera_tilenum = combined_tilenum
        self._compute_correspondence_errors()
        return True
    
    def filter_sources(self) -> None:
        print("filter_sources: not implemented for RegistrationPairFinder")
    
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
            assert self.results
            self.results.minCorrespondence[0] = 0.0
            self.results.minCorrespondenceSigma[0] = 0
            self.results.minCorrespondenceCount[0] = 1
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
            self.per_camera_nparray_distances[cam_i] = distances
            distances = self._filter_infinites(distances)
            histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
            cumsum = np.cumsum(histogram)
            totPoints = cumsum[-1]
            totOtherPoints = self._kdtree_count(dst_kdtree)
            normsum = cumsum / totPoints
            plot_label = f"{cam_tilenum} ({totPoints} to {totOtherPoints})"
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
            assert self.results
            self.results.minCorrespondence[0] = 0.0
            self.results.minCorrespondenceSigma[0] = 0
            self.results.minCorrespondenceCount[0] = 1
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
            self.per_camera_nparray_distances[cam_i] = distances
            distances = self._filter_infinites(distances)
            histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
            cumsum = np.cumsum(histogram)
            totPoints = cumsum[-1]
            totOtherPoints = self._kdtree_count(dst_kdtree)
            normsum = cumsum / totPoints
            plot_label = f"{cam_tilenum} ({totPoints} to {totOtherPoints})"
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
    RegistrationPairFinder
]

HELP_ANALYZER_ALGORITHMS = """
The analyzer algorithm looks at a source point cloud and a target point cloud and tries to determine how well they are aligned.

The following analyzer algorithms are available:
""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_ANALYZER_ALGORITHMS])