
from typing import List, Optional, Any, Tuple
import math
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

    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        return self.per_camera_tilenum[cam_index]

    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        for i in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[i] == tilenum:
                return i
        assert False, f"Tilenum {tilenum} not known"

    def camera_count(self):
        assert len(self.per_camera_tilenum) == len(self.per_camera_pointclouds)
        return len(self.per_camera_tilenum)
    
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        """Seve the resulting plot"""
        assert False

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

    def get_ordered_results(self) -> List[Tuple[int, float, float]]:
        """Returns a list of tuples (cameraNumber, correspondenceError, weight), ordered by weight (highest first)
        
        This is the order in which the camera re-registration should be attempted.
        """
        return []
    
class RegistrationAnalyzerOneToAll(RegistrationAnalyzer):

    # See comment in _compute_corrspondences()
    BIN_VALUE_DECREASE_FACTOR = 0.5

    def run(self, target: Optional[int]=None) -> None:
        """Run the algorithm"""
        assert target is None
        assert len(self.per_camera_pointclouds) > 1
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
