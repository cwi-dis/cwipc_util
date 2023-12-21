
from typing import List, Optional, Any, Tuple
import math
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from .. import cwipc_wrapper, cwipc_tilefilter
from .abstract import *
from .util import get_tiles_used, BaseAlgorithm

class RegistrationAnalyzer(AnalysisAlgorithm, BaseAlgorithm):
    """Analyzes how good pointclouds are registered.

    Create the registrator, add pointclouds, run the algorithm, inspect the results.
    Attributes want_plot, histogram_bincount and label can be changed before running.

    """

    # See comment in _compute_corrspondences()
    BIN_VALUE_DECREASE_FACTOR = 0.5

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.histogram_bincount = 400
        self.label : Optional[str] = None

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
        nCamera = len(self.per_camera_pointclouds)
        self.per_camera_histograms : List[Any] = [None] * nCamera
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            distances, _ = self.per_camera_kdtree_others[cam_i].query(self.per_camera_points_nparray[cam_i])
            histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
            cumsum = np.cumsum(histogram)
            totPoints = cumsum[-1]
            totOtherPoints = self.per_camera_kdtree_others[cam_i].data.shape[0]
            normsum = cumsum / totPoints
            plot_label = f"{cam_tilenum} ({totPoints} points to {totOtherPoints})"
            self.per_camera_histograms[cam_i] = (histogram, edges, cumsum, normsum, plot_label)
        self._compute_correspondence_errors()
        return True
    
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        """Seve the resulting plot"""
        # xxxjack This uses the stateful pyplot API. Horrible.
        if not filename and not show:
            return
        nCamera = len(self.per_camera_pointclouds)
        self.plot_fig, self.plot_ax = plt.subplots()
        for cam_i in range(nCamera):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            (histogram, edges, cumsum, normsum, plot_label) = self.per_camera_histograms[cam_i]
            if cumulative:
                self.plot_ax.plot(edges[1:], normsum, label=plot_label)
            else:
                self.plot_ax.plot(edges[1:], histogram, label=plot_label)
        corr_box_text = "Correspondence error:\n"
        for cam_i in range(len(self.correspondence_errors)):
            cam_tilenum = self.per_camera_tilenum[cam_i]
            corr_box_text += f"\n{cam_tilenum}: {self.correspondence_errors[cam_i]:.4f}"
        title = "Cumulative" if cumulative else "Histogram of"
        title = title + " point distances between camera and all others"
        if self.label:
            title = self.label + "\n" + title
        plt.title(title)
        props = dict(boxstyle='round', facecolor='white', alpha=0.5)
        self.plot_ax.text(0.98, 0.1, corr_box_text, transform=self.plot_ax.transAxes, fontsize='small', verticalalignment='bottom', horizontalalignment="right", bbox=props)
        self.plot_ax.legend()
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
        self.per_camera_points_nparray = [
            self._get_nparray_for_pc(cam_pc) for cam_pc in self.per_camera_pointclouds
        ]
        # Create the corresponding kdtrees, which consists of all points _not_ in this cloud

        self.per_camera_kdtree_others : List[scipy.spatial.KDTree] = []
        for cloud in self.per_camera_points_nparray:
            other_points = None
            for other_cloud in self.per_camera_points_nparray:
                if other_cloud is cloud:
                    continue
                if other_points is None:
                    other_points = other_cloud
                else:
                    other_points = np.concatenate([other_points, other_cloud])
            assert not other_points is None
            kdtree_others : scipy.spatial.KDTree = scipy.spatial.KDTree(other_points)  # type: ignore
            assert kdtree_others
            self.per_camera_kdtree_others.append(kdtree_others)

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
