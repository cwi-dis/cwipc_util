
from typing import List, Optional, Any
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from ..util import cwipc_wrapper, cwipc_tilefilter

class RegistrationAnalyzer:
    """Analyzes how good pointclouds are registered.

    Create the registrator, add pointclouds, run the algorithm, inspect the results.
    Attributes want_plot, histogram_bincount and label can be changed before running.

    xxxjack No way yet to get simple numbers, after running the algorithm.
    """

    def __init__(self):
        self.want_plot = False
        self.histogram_bincount = 400
        self.label : Optional[str] = None
        self.per_camera_tilenum : List[int] = []
        self.per_camera_pointclouds : List[cwipc_wrapper] = []

    def add_pointcloud(self, pc : cwipc_wrapper):
        """Add a pointcloud to be used during the algorithm run"""
        self.per_camera_tilenum.append(1000+len(self.per_camera_pointclouds))
        self.per_camera_pointclouds.append(pc)
        
    def add_tiled_pointcloud(self, pc : cwipc_wrapper):
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        for tilemask in [1,2,4,8,16,32,64,128]:
            tiled_pc = self._get_pc_for_cam(pc, tilemask)
            if tiled_pc == None:
                continue
            if tiled_pc.count() == 0:
                continue
            self.per_camera_pointclouds.append(tiled_pc)
            self.per_camera_tilenum.append(tilemask)

    def run(self):
        """Run the algorithm"""
        assert len(self.per_camera_pointclouds) > 1
        self._prepare()
        nCamera = len(self.per_camera_kdtree)

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
                if self.want_plot:
                    plt.plot(edges[1:], normsum, label=f"{cam_i} - {cam_j} ({totPoints} points)")
        if self.want_plot:
            title = "Cumulative point distances between all camera pairs"
            if self.label:
                title = self.label + "\n" + title
            plt.title(title)
            plt.legend()

    def save_plot(self, png_filename : str, show : bool = False):
        """Seve the resulting plot"""
        # xxxjack This uses the stateful pyplot API. Horrible.
        assert self.want_plot
        if png_filename:
            plt.savefig(png_filename)
        if show:
            plt.show()

    def _prepare(self):
        self.per_camera_points_nparray = [
            self._get_nparray_for_pc(cam_pc) for cam_pc in self.per_camera_pointclouds
        ]
        # Create the corresponding kdtrees
        self.per_camera_kdtree = [
            scipy.spatial.KDTree(points) for points in self.per_camera_points_nparray
        ]
   
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
