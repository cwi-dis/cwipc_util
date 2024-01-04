
from typing import List, Optional, Any, Tuple, cast
import math
import copy
import numpy as np
from numpy.typing import NDArray
import scipy.spatial
from matplotlib import pyplot as plt
from .. import cwipc_wrapper, cwipc_tilefilter
from .abstract import *
from .util import get_tiles_used, BaseAlgorithm

KD_TREE_TYPE = scipy.spatial.KDTree
PLOT_COLORS = ["r", "g", "b", "y", "m", "c", "orange", "lime"] # 8 colors. First 4 match cwipc_tilecolor().

class BaseRegistrationAnalyzer(AnalysisAlgorithm, BaseAlgorithm):
    """Analyzes how good pointclouds are registered.

    Create the registrator, add pointclouds, run the algorithm, inspect the results.
    Attributes histogram_bincount and label can be changed before running.

    This is the base class, see the subclasses for various different algorithms.
    """

    # See comment in _compute_corrspondences()
    BIN_VALUE_DECREASE_FACTOR = 0.5
    
    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.histogram_bincount = 400
        self.eps = 0.0
        self.distance_upper_bound = 10.0
        self.plot_label : Optional[str] = None
        self.plot_title = "Unknown RegistrationAnalyzer"
        self.per_camera_nparray : List[NDArray[Any]]= []    # numpy array of all points in this tile
        self.per_camera_nparray_others : List[NDArray[Any]]= [] # numpy array of all points in all other tiles
        self.per_camera_kdtree : List[KD_TREE_TYPE] = []    # kdtree of this tile
        self.per_camera_kdtree_others : List[KD_TREE_TYPE] = [] # kdtree of all other tiles combined
        self.per_camera_histograms : List[Tuple[NDArray[Any], NDArray[Any], NDArray[Any], NDArray[Any], str]]

    def run(self, target: Optional[int]=None) -> bool:
        assert False


    def _kdtree_get_distances_to_points(self, tree : KD_TREE_TYPE, points : NDArray[Any]) -> NDArray[Any]:
        distances, _ = tree.query(points, eps=self.eps, distance_upper_bound=self.distance_upper_bound)
        bitmap = np.isfinite(distances)
        filtered_distances = distances[bitmap]
        return filtered_distances
    
    def _kdtree_count(self, tree : KD_TREE_TYPE) -> int:
        return tree.data.shape[0]

    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        """Seve the resulting plot"""
        # xxxjack This uses the stateful pyplot API. Horrible.
        if not filename and not show:
            return
        nCamera = len(self.per_camera_pointclouds)
        plot_fig, plot_ax = plt.subplots()
        ax_cum = None
        if cumulative:
            ax_cum = plot_ax.twinx()
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            (histogram, edges, cumsum, normsum, plot_label) = self.per_camera_histograms[cam_i]
            plot_ax.plot(edges[1:], histogram, label=plot_label, color=PLOT_COLORS[cam_i])
            if cumulative:
                assert ax_cum
                ax_cum.plot(edges[1:], normsum, linestyle="dotted", label="_nolegend_", color=PLOT_COLORS[cam_i])
        corr_box_text = "Correspondence error:\n"
        for cam_i in range(len(self.correspondence_errors)):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            corr_box_text += f"\n{cam_tilenum}: {self.correspondence_errors[cam_i]:.4f}"
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
   
    def get_ordered_results(self) -> List[Tuple[int, float, float]]:
        """Returns a list of tuples (cameraNumber, correspondenceError, weight), ordered by weight (highest first)
        
        This is the order in which the camera re-registration should be attempted.
        """
        rv = []
        for camnum in range(len(self.correspondence_errors)):
            # Option 1: Use the correspondence as-is
            #weight = self.correspondences[camnum]
            # Option 2: multiply by the number of points that were matched
            #weight = self.correspondences[camnum]*self.below_correspondence_counts[camnum]
            # option 3: multiply by the square root of the number of matched points
            #weight = self.correspondences[camnum]*math.sqrt(self.below_correspondence_counts[camnum])
            # option 4: multiply by the log of the number of matched points
            weight = self.correspondence_errors[camnum]*math.log(self.below_correspondence_error_counts[camnum])
            rv.append((self.per_camera_tilenum[camnum], self.correspondence_errors[camnum], weight))
        rv.sort(key=lambda t:-t[2])
        return rv
    
    def _prepare(self):
        assert False

    def _prepare_nparrays(self):
        assert self.per_camera_nparray == []
        self.per_camera_nparray = [
            self._get_nparray_for_pc(cam_pc) for cam_pc in self.per_camera_pointclouds
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
        nCamera = len(self.per_camera_histograms)
        self.correspondence_errors : List[float] = []
        self.below_correspondence_error_counts : List[int] = []
        for histogram, edges, cumsum, normsum, plot_label in self.per_camera_histograms:
            # Find the fullest bin, and the corresponding value
            max_bin_index = int(np.argmax(histogram))
            max_bin_value = histogram[max_bin_index]
            # Now we traverse the histogram from here, until we get to a bin that has less than half this number of points
            for corr_bin_index in range(max_bin_index, len(histogram)):
                if histogram[corr_bin_index] <= max_bin_value * self.BIN_VALUE_DECREASE_FACTOR:
                    break
            else:
                corr_bin_index = max_bin_index+1
            # Now corr_bin_index is *one past* our expected correspondence is
            # Note that this is important for the edge case (when all points are exactly aligned)
            corr_bin_index -= 1
            corr = edges[corr_bin_index]
            below_corr_count = cumsum[corr_bin_index]
            self.correspondence_errors.append(corr)
            self.below_correspondence_error_counts.append(below_corr_count)

class RegistrationAnalyzer(BaseRegistrationAnalyzer):
    """Compute registration by checking the distances from every point in this tile pointcloud to the nearest in any other pointcloud"""

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
                ([1], [0,1], [1], [1], f"single tile {self.per_camera_tilenum[0]}")
            ]
            self.correspondence_errors : List[float] = [0.0]
            self.below_correspondence_error_counts : List[int] = [1]
            return True
        self._prepare()
        # Ensure we have the arrays and kdtrees we need
        assert self.per_camera_nparray
        assert self.per_camera_kdtree_others

        nCamera = len(self.per_camera_pointclouds)
        self.per_camera_histograms : List[Any] = [None] * nCamera
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
            self.per_camera_histograms[cam_i] = (histogram, edges, cumsum, normsum, plot_label)
        self._compute_correspondence_errors()
        return True

class RegistrationAnalyzerFiltered(RegistrationAnalyzer):
    """Compute registration by checking the distances from every point in this tile pointcloud to the nearest in any other pointcloud.
    
    All point clouds are first filtered to remove the points that are unlikely to have a match (no make the pointclouds and the kdtrees smaller)
    """
    
    def _prepare(self):
        self._prepare_nparrays()
        self._prepare_kdtrees()
        self._filter_nparrays()
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
                    

class RegistrationAnalyzerReverse(RegistrationAnalyzer):
    """Compute registration by checking the distances from every point in any other tile pointcloud to the nearest in this tile pointcloud"""

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
                ([1], [0,1], [1], [1], f"single tile {self.per_camera_tilenum[0]}")
            ]
            self.correspondence_errors : List[float] = [0.0]
            self.below_correspondence_error_counts : List[int] = [1]
            return True
        self._prepare()
        # Ensure we have the arrays and kdtrees we need
        assert self.per_camera_nparray_others
        assert self.per_camera_kdtree

        nCamera = len(self.per_camera_pointclouds)
        self.per_camera_histograms : List[Any] = [None] * nCamera
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
            self.per_camera_histograms[cam_i] = (histogram, edges, cumsum, normsum, plot_label)
        self._compute_correspondence_errors()
        return True
    
class RegistrationAnalyzerFilteredReverse(RegistrationAnalyzerReverse):
    """Compute registration by checking the distances from every point in any other tile pointcloud to the nearest in this tile pointcloud.
    
    All point clouds are first filtered to remove the points that are unlikely to have a match (no make the pointclouds and the kdtrees smaller)
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
                    
