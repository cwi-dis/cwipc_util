
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from .. import cwipc_wrapper, cwipc_tilefilter
from .abstract import *

class RegistrationAnalyzer(RegistrationAlgorithm):
    """Analyzes how good pointclouds are registered.

    Create the registrator, add pointclouds, run the algorithm, inspect the results.
    Attributes want_plot, histogram_bincount and label can be changed before running.

    """

    def __init__(self):
        self.want_cumulative_plot = False
        self.want_histogram_plot = False
        self.histogram_bincount = 400
        self.label : Optional[str] = None
        self.per_camera_tilenum : List[int] = []
        self.per_camera_pointclouds : List[cwipc_wrapper] = []

    def add_pointcloud(self, pc : cwipc_wrapper) -> int:
        """Add a pointcloud to be used during the algorithm run"""
        tilenum = 1000+len(self.per_camera_pointclouds)
        self.per_camera_tilenum.append(tilenum)
        self.per_camera_pointclouds.append(pc)
        return tilenum
        
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        for tilemask in [1,2,4,8,16,32,64,128]:
            tiled_pc = self._get_pc_for_cam(pc, tilemask)
            if tiled_pc == None:
                continue
            if tiled_pc.count() == 0:
                continue
            self.per_camera_pointclouds.append(tiled_pc)
            self.per_camera_tilenum.append(tilemask)

    def save_plot(self, png_filename : str, show : bool = False):
        """Seve the resulting plot"""
        # xxxjack This uses the stateful pyplot API. Horrible.
        assert self.want_cumulative_plot or self.want_histogram_plot
        if png_filename:
            plt.savefig(png_filename)
        if show:
            plt.show()
   
    def _get_pc_for_cam(self, pc : cwipc_wrapper, tilemask : int) -> Optional[cwipc_wrapper]:
        rv = cwipc_tilefilter(pc, tilemask)
        if rv.count() != 0:
            return rv
        rv.free()
        return None

    def _get_nparray_for_pc(self, pc : cwipc_wrapper):
        # Get the points (as a cwipc-style array) and convert them to a NumPy array-of-structs
        pointarray = np.ctypeslib.as_array(pc.get_points())
        # Extract the relevant fields (X, Y, Z coordinates)
        xyzarray = pointarray[['x', 'y', 'z']]
        # Turn this into an N by 3 2-dimensional array
        nparray = np.column_stack([xyzarray['x'], xyzarray['y'], xyzarray['z']])
        return nparray

class RegistrationAnalyzerOneToOne(RegistrationAnalyzer):

    def run(self, target: Optional[int]=None) -> None:
        """Run the algorithm"""
        assert target is None
        assert len(self.per_camera_pointclouds) > 1
        self._prepare()
        nCamera = len(self.per_camera_pointclouds)

        self.inter_camera_histograms : List[List[Any]] = [[None] * nCamera] * nCamera
        for cam_i in range(nCamera):
            for cam_j in range(cam_i+1, nCamera):
                distances_forward, _ = self.per_camera_kdtree[cam_j].query(self.per_camera_points_nparray[cam_i])
                distances_backward, _ = self.per_camera_kdtree[cam_i].query(self.per_camera_points_nparray[cam_j])
                distances = np.concatenate([distances_forward, distances_backward])
                histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
                cumsum = np.cumsum(histogram)
                totPoints = cumsum[-1]
                normsum = cumsum / totPoints
                self.inter_camera_histograms[cam_i][cam_j] = (normsum, edges)
                if self.want_cumulative_plot:
                    plt.plot(edges[1:], normsum, label=f"{cam_i} - {cam_j} ({totPoints} points)")
                if self.want_histogram_plot:
                    plt.plot(edges[1:], histogram, label=f"{cam_i} - {cam_j} ({totPoints} points)")
        if self.want_cumulative_plot or self.want_histogram_plot:
            title = "Cumulative" if self.want_cumulative_plot else "Histogram of"
            title = title + " point distances between all camera pairs"
            if self.label:
                title = self.label + "\n" + title
            plt.title(title)
            plt.legend()

    def _prepare(self):
        self.per_camera_points_nparray = [
            self._get_nparray_for_pc(cam_pc) for cam_pc in self.per_camera_pointclouds
        ]
        # Create the corresponding kdtrees
        self.per_camera_kdtree = [
            scipy.spatial.KDTree(points) for points in self.per_camera_points_nparray
        ]

class RegistrationAnalyzerOneToAll(RegistrationAnalyzer):

    # See comment in _compute_corrspondences()
    BIN_VALUE_DECREASE_FACTOR = 0.5

    def run(self, target: Optional[int]=None) -> None:
        """Run the algorithm"""
        assert target is None
        assert len(self.per_camera_pointclouds) > 1
        self._prepare()
        nCamera = len(self.per_camera_pointclouds)
        self.plot_fig, self.plot_ax = plt.subplots()
        self.per_camera_histograms : List[Any] = [None] * nCamera
        for cam_i in range(nCamera):
            distances, _ = self.per_camera_kdtree_others[cam_i].query(self.per_camera_points_nparray[cam_i])
            histogram, edges = np.histogram(distances, bins=self.histogram_bincount)
            cumsum = np.cumsum(histogram)
            totPoints = cumsum[-1]
            totOtherPoints = self.per_camera_kdtree_others[cam_i].data.shape[0]
            normsum = cumsum / totPoints
            self.per_camera_histograms[cam_i] = (histogram, edges, cumsum)
            if self.want_cumulative_plot:
                self.plot_ax.plot(edges[1:], normsum, label=f"{cam_i} ({totPoints} points to {totOtherPoints})")
            if self.want_histogram_plot:
                self.plot_ax.plot(edges[1:], histogram, label=f"{cam_i} ({totPoints} points to {totOtherPoints})")
        self._compute_correspondences()
        corr_box_text = "Correspondence error:\n"
        for i in range(len(self.correspondences)):
            corr_box_text += f"\n{i}: {self.correspondences[i]:.3f}"

        if self.want_cumulative_plot or self.want_histogram_plot:
            title = "Cumulative" if self.want_cumulative_plot else "Histogram of"
            title = title + " point distances between camera and all others"
            if self.label:
                title = self.label + "\n" + title
            plt.title(title)
            props = dict(boxstyle='round', facecolor='white', alpha=0.5)
            self.plot_ax.text(0.98, 0.1, corr_box_text, transform=self.plot_ax.transAxes, fontsize='small', verticalalignment='bottom', horizontalalignment="right", bbox=props)
            self.plot_ax.legend()

    def get_ordered_results(self) -> List[Tuple[int, float, float]]:
        """Returns a list of tuples (cameraNumber, correspondenceError, weight), ordered by weight (highest first)
        
        This is the order in which the camera re-registration should be attempted.
        """
        rv = []
        for camnum in range(len(self.correspondences)):
            weight = self.correspondences[camnum]*self.below_correspondence_counts[camnum]
            rv.append((self.per_camera_tilenum[camnum], self.correspondences[camnum], weight))
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

    def _compute_correspondences(self):
        nCamera = len(self.per_camera_histograms)
        self.correspondences : List[float] = []
        self.below_correspondence_counts : List[int] = []
        for histogram, edges, cumsum in self.per_camera_histograms:
            # Find the fullest bin, and the corresponding value
            max_bin_index = int(np.argmax(histogram))
            max_bin_value = histogram[max_bin_index]
            # Now we traverse the histogram from here, until we get to a bin that has less than half this number of points
            for corr_bin_index in range(max_bin_index, len(histogram)):
                if histogram[corr_bin_index] < max_bin_value * self.BIN_VALUE_DECREASE_FACTOR:
                    break
            else:
                corr_bin_index = max_bin_index
            # Now corr_bin_index is where our expected correspondence is
            corr = edges[corr_bin_index]
            below_corr_count = cumsum[corr_bin_index]
            self.correspondences.append(corr)
            self.below_correspondence_counts.append(below_corr_count)
