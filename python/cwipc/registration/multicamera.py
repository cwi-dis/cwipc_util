
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt

from cwipc.registration.abstract import RegistrationTransformation
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_downsample, cwipc_write
from .abstract import *
from .util import transformation_identity
from .analyze import RegistrationAnalyzer, RegistrationAnalyzer
from .fine import RegistrationComputer_ICP_Point2Point

class MultiCamera(MultiAlignmentAlgorithm):
    """Align multiple cameras.
    """

    def __init__(self):
        self.precision_threshold = 0.001 # Don't attempt to re-align better than 1mm

        self.untiled_pointclouds : List[cwipc_wrapper] = []
        self.tiled_pointclouds : List[cwipc_wrapper] = []
        self.transformations : List[RegistrationTransformation] = []
        self.original_transformations : List[RegistrationTransformation] = []
        self.change : List[float] = []
        self.results : List[Tuple[int, float, float]] = []
        self.cellsize_factor = 4 # math.sqrt(2)
        self.proposed_cellsize = 0

        self.analyzer_class = RegistrationAnalyzer
        self.aligner_class = RegistrationComputer_ICP_Point2Point

        self.analyzer : Optional[AnalysisAlgorithm] = None
        self.aligner : Optional[AlignmentAlgorithm] = None

        self.verbose = False
        self.show_plot = False

    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        assert False

    def add_pointcloud(self, pc : cwipc_wrapper) -> int:
        """Add a pointcloud to be used during the algorithm run"""
        tilenum = 1000+len(self.untiled_pointclouds)
        #self.per_camera_tilenum.append(tilenum)
        self.untiled_pointclouds.append(pc)
        return tilenum
        
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        self.tiled_pointclouds.append(pc)

    def camera_count(self) -> int:
        assert self.analyzer
        return self.analyzer.camera_count()
    
    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        assert self.analyzer
        return self.analyzer.tilenum_for_camera_index(cam_index)

    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        assert self.analyzer
        return self.analyzer.camera_index_for_tilenum(tilenum)
    
    def _get_pc_for_cam(self, pc : cwipc_wrapper, tilemask : int) -> Optional[cwipc_wrapper]: # xxxjack needed?
        rv = cwipc_tilefilter(pc, tilemask)
        if rv.count() != 0:
            return rv
        rv.free()
        return None

    def _get_nparray_for_pc(self, pc : cwipc_wrapper): # xxxjack needed?
        # Get the points (as a cwipc-style array) and convert them to a NumPy array-of-structs
        pointarray = np.ctypeslib.as_array(pc.get_points())
        # Extract the relevant fields (X, Y, Z coordinates)
        xyzarray = pointarray[['x', 'y', 'z']]
        # Turn this into an N by 3 2-dimensional array
        nparray = np.column_stack([xyzarray['x'], xyzarray['y'], xyzarray['z']])
        return nparray
    
    def _prepare_analyze(self):
        self.analyzer = None
        assert self.aligner_class
        self.analyzer = self.analyzer_class()
        for pc in self.untiled_pointclouds:
            self.analyzer.add_pointcloud(pc)
        for pc in self.tiled_pointclouds:
            self.analyzer.add_tiled_pointcloud(pc)

    def _prepare_compute(self):
        self.aligner = None
        self.aligner = self.aligner_class()
        for pc in self.untiled_pointclouds:
            self.aligner.add_pointcloud(pc)
        for pc in self.tiled_pointclouds:
            self.aligner.add_tiled_pointcloud(pc)

    def run(self) -> bool:
        """Run the algorithm"""
        assert len(self.untiled_pointclouds) == 0 # Algorithm not implemented fully for separate ply files
        assert len(self.tiled_pointclouds) == 1
        # Initialize the analyzer
        self._prepare_analyze()
        assert self.analyzer
        assert self.analyzer.camera_count() > 0
        # Initialize matrices, if not done already (by our caller)
        if len(self.transformations) == 0:
            for i in range(self.analyzer.camera_count()):
                self.transformations.append(transformation_identity())
        self.original_transformations = copy.deepcopy(self.transformations)
        # Run the analyzer for the first time, on the original pointclouds.
        self.analyzer.run()
        self.results = self.analyzer.get_ordered_results()
        camnum_to_fix, correspondence, total_correspondence = self._get_next_candidate()
        if self.verbose:
            print(f"Before: overall correspondence error {total_correspondence}. Per-camera correspondence, ordered worst-first:")
            for _camnum, _correspondence, _weight in self.results:
                print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
        if self.show_plot:
            self.analyzer.plot(show=True)
        stepnum = 1
        while camnum_to_fix != None:
            if self.verbose:
                print(f"Step {stepnum}: camera {camnum_to_fix}, correspondence error {correspondence}, overall correspondence error {total_correspondence}")
            # Prepare the registration computer
            self._prepare_compute()
            assert self.aligner
            self.aligner.set_correspondence(correspondence)
            self.aligner.run(camnum_to_fix)
            # Save resultant pointcloud
            old_pc = self.tiled_pointclouds[0]
            new_pc = self.aligner.get_result_pointcloud_full()
            old_pc.free()
            self.tiled_pointclouds[0] = new_pc
            # Apply new transformation (to the left of the old one)
            cam_index = self.aligner.camera_index_for_tilenum(camnum_to_fix)
            old_transform = self.transformations[cam_index]
            this_transform = self.aligner.get_result_transformation()
            new_transform = np.matmul(this_transform, old_transform)
            # print(f"xxxjack camnum_to_fix={camnum_to_fix}, camindex={cam_index}, new_transform={new_transform}")
            self.transformations[cam_index] = new_transform
            # Re-initialize analyzer
            self._prepare_analyze()
            self.analyzer.run()
            self.results = self.analyzer.get_ordered_results()
            if self.verbose:
                print(f"Step {stepnum}: per-camera correspondence, ordered worst-first:")
                for _camnum, _correspondence, _weight in self.results:
                    print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
            # See results, and whether it's worth it to do another step
            old_camnum_to_fix = camnum_to_fix
            old_correspondence = correspondence
            old_total_correspondence = total_correspondence
            camnum_to_fix, correspondence, total_correspondence = self._get_next_candidate()
            # This stop-condition can be improved, in various ways:
            # - If total_correspondence is worse than previously we should undo this step and try another camera
            # - We may also want to try another (more expensive) algorithm
            if camnum_to_fix == old_camnum_to_fix and correspondence >= old_correspondence:
                break
            stepnum += 1
        self.proposed_cellsize = total_correspondence*self.cellsize_factor
        if self.verbose:
            print(f"After {stepnum} steps: overall correspondence error {total_correspondence}. Per-camera correspondence, ordered worst-first:")
            for _camnum, _correspondence, _weight in self.results:
                print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
        if self.show_plot:
            self.analyzer.plot(show=True)
        self._compute_change()
        if self.verbose:
            for cam_index in range(len(self.change)):
                print(f"\tcamindex={cam_index}, change={self.change[cam_index]}")
        self._compute_new_tiles()
        return True

    def _get_next_candidate(self) -> Tuple[Optional[int], float, float]:
        camnum_to_fix = None
        correspondence = self.results[0][1]
        for i in range(len(self.results)):
            if self.results[i][1] > self.precision_threshold:
                camnum_to_fix = self.results[i][0]
                correspondence = self.results[i][1]
                break
        w_sum = 0
        c_sum = 0
        for res in self.results:
            c_sum += res[1] * res[2]
            w_sum += res[2]
        if w_sum == 0:
            w_sum = 1
        return camnum_to_fix, correspondence, c_sum / w_sum

    def _compute_change(self):
        for cam_index in range(len(self.transformations)):
            orig_transform : np.ndarray = self.original_transformations[cam_index] # type: ignore
            orig_transform_inv = np.linalg.inv(orig_transform)
            new_transform : np.ndarray = self.transformations[cam_index] # type: ignore
            new_transform_inv = np.linalg.inv(new_transform)
            #if self.verbose:
            #    print(f"camindex={cam_index} old: {orig_transform}")
            #    print(f"camindex={cam_index} new: {new_transform}")
            # Compute how far the point cloud moved
            # we take four points and compute the average move
            total_delta = 0.0
            for point in [
                    np.array([0, 0, 0, 1]),
                    np.array([0, 2, 0, 1]),
                    np.array([[-0.5, 1, 0, 1]]),
                    np.array([0, 1, 0.5, 1]),
                    ]:
                tmp = orig_transform_inv @ point.transpose()
                new_point = (new_transform @ tmp).transpose()
                delta : float = float(np.linalg.norm(point - new_point))
                total_delta += delta
            self.change.append(total_delta / 4)

    def _compute_new_tiles(self):
        pc = self.tiled_pointclouds[0]
        ntiles_orig = len(self.transformations)
        pc_new = cwipc_downsample(pc, self.proposed_cellsize)
        cwipc_write("tiled.ply", pc_new)
        if self.verbose:
            print(f"Voxelizing with {self.proposed_cellsize}: point count {pc_new.count()}, was {pc.count()}")
        pointcounts = []
        for i in range(2**ntiles_orig):
            pc_tile = cwipc_tilefilter(pc_new, i)
            pointcount = pc_tile.count()
            pc_tile.free()
            pointcounts.append(pointcount)
        if self.verbose:
            print(f"Pointcounts per tile, after voxelizing:")
            for i in range(len(pointcounts)):
                print(f"\ttile {i}: {pointcounts[i]}")

    def get_result_transformations(self) -> List[RegistrationTransformation]:
        return self.transformations