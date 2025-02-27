from abc import ABC, abstractmethod
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from cwipc import cwipc_wrapper, cwipc_from_packet

from cwipc.registration.abstract import RegistrationTransformation
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_downsample, cwipc_write
from .abstract import *
from .util import transformation_identity
from .analyze import RegistrationAnalyzerNoOp
from .fine import DEFAULT_ALIGNMENT_ALGORITHM

class MultiCameraBase(MultiAlignmentAlgorithm):
    """Base class for multi-camera alignment algorithms.
    """
    current_pointcloud : Optional[cwipc_wrapper]
    current_pointcloud_is_new : bool
    transformations : List[RegistrationTransformation]
    original_transformations : List[RegistrationTransformation]
    results : List[Tuple[int, float, float]]
    analyzer_class : Optional[AnalysisAlgorithmFactory]
    analyzer : Optional[AnalysisAlgorithm]
    aligner_class : Optional[AlignmentAlgorithmFactory]
    aligner : Optional[AlignmentAlgorithm]
    verbose : bool
    show_plot : bool

    def __init__(self):
        self.current_pointcloud = None
        self.current_pointcloud_is_new = False
        self.transformations  = []
        self.original_transformations = []
        self.results = []

        self.analyzer_class = None
        self.analyzer = None
        self.aligner_class = None
        self.aligner = None

        self.verbose = False
        self.show_plot = False


    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        assert False


    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        self.current_pointcloud = pc
        self.current_pointcloud_is_new = False

    def get_pointcloud_for_tilenum(self, tilenum : int) -> cwipc_wrapper:
        """Returns the point cloud for this tilenumber"""
        assert self.current_pointcloud
        rv = cwipc_tilefilter(self.current_pointcloud, tilenum)
        return rv
    
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
    
    @abstractmethod
    def _prepare_analyze(self):
        assert False

    @abstractmethod
    def _prepare_compute(self):
        assert False

    def set_original_transform(self, cam_index : int, matrix : RegistrationTransformation) -> None:
        self._prepare_analyze()
        assert self.camera_count() > 0
        if len(self.transformations) == 0:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())
        self.transformations[cam_index] = matrix

    @abstractmethod
    def run(self) -> bool:
        """Run the algorithm"""
        assert False

    def get_result_transformations(self) -> List[RegistrationTransformation]:
        return self.transformations
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        assert self.current_pointcloud
        if not self.current_pointcloud_is_new:
            # Do a deep-copy, so our caller can free the pointcloud it passed to us
            self.current_pointcloud = cwipc_from_packet(self.current_pointcloud.get_packet())
            self.current_pointcloud_is_new = True
        return self.current_pointcloud
  
class MultiCameraNoOp(MultiCameraBase):
    """No-op algorithm for testing purposes.
    """
    def __init__(self):
        super().__init__()

    def _prepare_analyze(self):
        self.analyzer = RegistrationAnalyzerNoOp()
        assert self.current_pointcloud
        self.analyzer.add_tiled_pointcloud(self.current_pointcloud)

    def _prepare_compute(self):
        self.aligner = None
        assert self.current_pointcloud

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.current_pointcloud
        return True

class MultiCameraOneToAllOthers(MultiCameraBase):
    """Align multiple cameras. Every step, one camera is aligned to all others.
    Every step, we pick the camera with the best chances to make the biggest change.
    """
    precision_threshold : float
    change : List[float]
    cellsize_factor : float
    proposed_cellsize : float


    def __init__(self):
        super().__init__()
        self.precision_threshold = 0.001 # Don't attempt to re-align better than 1mm
        self.change = []
        self.cellsize_factor = math.sqrt(2)
        self.proposed_cellsize = 0
    
    def _prepare_analyze(self):
        self.analyzer = None
        assert self.analyzer_class
        if self.verbose:
            print(f"{__class__.__name__}: Use analyzer class {self.analyzer_class.__name__}")
        self.analyzer = self.analyzer_class()
        self.analyzer.verbose = self.verbose
        assert self.current_pointcloud
        self.analyzer.add_tiled_pointcloud(self.current_pointcloud)

    def _prepare_compute(self):
        self.aligner = None
        if not self.aligner_class:
            self.aligner_class = DEFAULT_ALIGNMENT_ALGORITHM
        if self.verbose:
            print(f"{__class__.__name__}: Use aligner class {self.aligner_class.__name__}")
        self.aligner = self.aligner_class()
        self.aligner.verbose = self.verbose
        assert self.current_pointcloud
        self.aligner.add_tiled_pointcloud(self.current_pointcloud)

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.current_pointcloud
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
        camnum_to_fix, correspondence, total_correspondence = self._get_next_candidate([])
        if self.verbose:
            print(f"{__class__.__name__}: Before: overall correspondence error {total_correspondence}. Per-camera correspondence, ordered worst-first:")
            for _camnum, _correspondence, _weight in self.results:
                print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
        if self.show_plot:
            self.analyzer.plot(show=True)
        stepnum = 1
        camnums_already_fixed = []
        while camnum_to_fix != None:
            if self.verbose:
                print(f"{__class__.__name__}: Step {stepnum}: camera {camnum_to_fix}, correspondence error {correspondence}, overall correspondence error {total_correspondence}")
            # Prepare the registration computer
            self._prepare_compute()
            assert self.aligner
            self.aligner.set_correspondence(correspondence)
            self.aligner.run(camnum_to_fix)
            camnums_already_fixed.append(camnum_to_fix)
            # Save resultant pointcloud
            old_pc = self.current_pointcloud
            new_pc = self.aligner.get_result_pointcloud_full()
            old_pc.free()
            self.current_pointcloud = new_pc
            self.current_pointcloud_is_new = True
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
                print(f"{__class__.__name__}: Step {stepnum}: per-camera correspondence, ordered worst-first:")
                for _camnum, _correspondence, _weight in self.results:
                    print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
            # See results, and whether it's worth it to do another step
            old_camnum_to_fix = camnum_to_fix
            old_correspondence = correspondence
            old_total_correspondence = total_correspondence
            camnum_to_fix, correspondence, total_correspondence = self._get_next_candidate(camnums_already_fixed)
            # This stop-condition can be improved, in various ways:
            # - If total_correspondence is worse than previously we should undo this step and try another camera
            # - We may also want to try another (more expensive) algorithm
            if camnum_to_fix == old_camnum_to_fix and correspondence >= old_correspondence-0.0001:
                if self.verbose:
                    print(f"{__class__.__name__}: Step {stepnum}: Giving up: went only from {old_correspondence} to {correspondence}")
                break
            stepnum += 1
        self.proposed_cellsize = total_correspondence*self.cellsize_factor
        if self.verbose:
            print(f"{__class__.__name__}: After {stepnum} steps: overall correspondence error {total_correspondence}. Per-camera correspondence, ordered worst-first:")
            for _camnum, _correspondence, _weight in self.results:
                print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
        if self.show_plot:
            self.analyzer.plot(show=True)
        self._compute_change()
        if self.verbose:
            print(f"{__class__.__name__}: Change in matrices after alignment:")
            for cam_index in range(len(self.change)):
                print(f"\tcamindex={cam_index}, change={self.change[cam_index]}")
        self._compute_new_tiles()
        return True

    def _get_next_candidate(self, camnums_already_fixed : List[int]) -> Tuple[Optional[int], float, float]:
        camnum_to_fix = None
        correspondence = self.results[0][1]
        for i in range(len(self.results)):
            if self.results[i][0] in camnums_already_fixed:
                continue
            if self.results[i][1] > self.precision_threshold:
                camnum_to_fix = self.results[i][0]
                correspondence = self.results[i][1]
                break
        if camnum_to_fix == None:
            # Apparently all cameras have been done at least once.
            camnum_to_fix = self.results[0][0]
            correspondence = self.results[0][1]
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
        assert self.current_pointcloud
        pc = self.current_pointcloud
        ntiles_orig = len(self.transformations)
        must_free_new = False
        if self.proposed_cellsize > 0:
            pc_new = cwipc_downsample(pc, self.proposed_cellsize)
            must_free_new = True
        else:
            pc_new = pc
            must_free_new = False
        if self.verbose:
            print(f"{__class__.__name__}: Voxelizing with {self.proposed_cellsize}: point count {pc_new.count()}, was {pc.count()}")
        pointcounts = []
        for i in range(2**ntiles_orig):
            pc_tile = cwipc_tilefilter(pc_new, i)
            pointcount = pc_tile.count()
            pc_tile.free()
            pointcounts.append(pointcount)
        if self.verbose:
            print(f"{__class__.__name__}: Pointcounts per tile, after voxelizing:")
            for i in range(len(pointcounts)):
                print(f"\ttile {i}: {pointcounts[i]}")
        if must_free_new:
            pc_new.free()
    

class MultiCameraIterative(MultiCameraBase):
    resultant_pointcloud : Optional[cwipc_wrapper]
    still_to_do : List[Tuple[int, float, float]]
    cellsize_factor : float
    proposed_cellsize : float
    change : List[float]

    def __init__(self):
        super().__init__()
        self.resultant_pointcloud = None
        self.still_to_do = []
        self.cellsize_factor = math.sqrt(2)
        self.proposed_cellsize = 0
        self.change = []

    def _prepare_analyze(self):
        self.analyzer = None
        assert self.analyzer_class
        if self.verbose:
            print(f"{__class__.__name__}: Use analyzer class {self.analyzer_class.__name__}")
        self.analyzer = self.analyzer_class()
        self.analyzer.verbose = self.verbose
        assert self.current_pointcloud
        self.analyzer.add_tiled_pointcloud(self.current_pointcloud)

    def _prepare_compute(self):
        self.aligner = None
        if not self.aligner_class:
            self.aligner_class = DEFAULT_ALIGNMENT_ALGORITHM
        if self.verbose:
            print(f"{__class__.__name__}: Use aligner class {self.aligner_class.__name__}")
        self.aligner = self.aligner_class()
        self.aligner.verbose = self.verbose
        assert self.current_pointcloud
        self.aligner.add_tiled_pointcloud(self.current_pointcloud)

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.current_pointcloud
        assert self.analyzer
        assert self.analyzer.camera_count() > 0
        # Initialize matrices, if not done already (by our caller)
        if len(self.transformations) == 0:
            for i in range(self.analyzer.camera_count()):
                self.transformations.append(transformation_identity())
        self.original_transformations = copy.deepcopy(self.transformations)
        # Run the analyzer for the first time, on the original pointclouds.
        self.analyzer.run()
        self.results = self.analyzer.get_ordered_results(weightstyle='match')
        self.still_to_do = self.results
        if self.verbose:
            print(f"{__class__.__name__}: Before:  Per-camera correspondence, ordered best-first:")
            for _camnum, _correspondence, _weight in self.results:
                print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
        if self.show_plot:
            self.analyzer.plot(show=True)
            
        self._select_first_pointcloud()
        while self.still_to_do:
            assert self.resultant_pointcloud
            camnum_to_fix, correspondence = self._select_next_pointcloud_index()
            if self.verbose:
                print(f"{__class__.__name__}: Aligning camera {camnum_to_fix}, corr={correspondence}, {self.resultant_pointcloud.count()} points in reference set")
            # Prepare the registration computer
            self._prepare_compute()
            assert self.aligner
            assert self.resultant_pointcloud
            self.aligner.set_correspondence(correspondence)
            self.aligner.set_reference_pointcloud(self.resultant_pointcloud)
            self.aligner.run(camnum_to_fix)
            # Save resultant pointcloud
            old_pc = self.resultant_pointcloud
            new_pc = self.aligner.get_result_pointcloud_full()
            old_pc.free()
            self.resultant_pointcloud = new_pc
            # Apply new transformation (to the left of the old one)
            cam_index = self.aligner.camera_index_for_tilenum(camnum_to_fix)
            old_transform = self.transformations[cam_index]
            this_transform = self.aligner.get_result_transformation()
            new_transform = np.matmul(this_transform, old_transform)
            self.transformations[cam_index] = new_transform
        # Finally, run the analyzer on the final result
        self.current_pointcloud = self.resultant_pointcloud
        self.current_pointcloud_is_new = True
        self._prepare_analyze()
        self.analyzer.run()
        self.results = self.analyzer.get_ordered_results()
        max_correspondence = 0
        for _, correspondence, _ in self.results:
            max_correspondence = max(max_correspondence, correspondence)
        self.proposed_cellsize = max_correspondence * self.cellsize_factor

        if self.verbose:
            print(f"{__class__.__name__}: After all cameras done. Per-camera correspondence, ordered worst-first:")
            for _camnum, _correspondence, _weight in self.results:
                print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
        if self.show_plot:
            self.analyzer.plot(show=True)
        self._compute_change()
        if self.verbose:
            print(f"{__class__.__name__}: Change in matrices after alignment:")
            for cam_index in range(len(self.change)):
                print(f"\tcamindex={cam_index}, change={self.change[cam_index]}")
        self._compute_new_tiles()
        return True
        # Run the analyzer for the first time, on the original pointclouds.

    def _select_first_pointcloud(self) -> None:
        assert self.resultant_pointcloud == None
        camNum, _, _ = self.still_to_do[0]
        if self.verbose:
            print(f"{__class__.__name__}: Select initial pointcloud: camera {camNum}")
        self.still_to_do = self.still_to_do[1:]
        pc = self.get_pointcloud_for_tilenum(camNum)
        # Do a deep-copy
        self.resultant_pointcloud = cwipc_from_packet(pc.get_packet())

    def _select_next_pointcloud_index(self) -> Tuple[int, float]:
        camNum, correspondence, _ = self.still_to_do[0]
        self.still_to_do = self.still_to_do[1:]
        if self.verbose:
            print(f"{__class__.__name__}: Select next pointcloud: camera {camNum}, correspondence {correspondence}")
        return camNum, correspondence

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
        assert self.current_pointcloud
        pc = self.current_pointcloud
        ntiles_orig = len(self.transformations)
        must_free_new = False
        if self.proposed_cellsize > 0:
            pc_new = cwipc_downsample(pc, self.proposed_cellsize)
            must_free_new = True
        else:
            pc_new = pc
            must_free_new = False
        if self.verbose:
            print(f"{__class__.__name__}: Voxelizing with {self.proposed_cellsize}: point count {pc_new.count()}, was {pc.count()}")
        pointcounts = []
        for i in range(2**ntiles_orig):
            pc_tile = cwipc_tilefilter(pc_new, i)
            pointcount = pc_tile.count()
            pc_tile.free()
            pointcounts.append(pointcount)
        if self.verbose:
            print(f"{__class__.__name__}: Pointcounts per tile, after voxelizing:")
            for i in range(len(pointcounts)):
                print(f"\ttile {i}: {pointcounts[i]}")
        if must_free_new:
            pc_new.free()
    

DEFAULT_FINE_ALIGNMENT_ALGORITHM = MultiCameraOneToAllOthers