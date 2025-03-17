from abc import ABC, abstractmethod
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from cwipc import cwipc_wrapper, cwipc_from_packet, cwipc_from_numpy_matrix

from cwipc.registration.abstract import RegistrationTransformation
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_downsample, cwipc_write
from .abstract import *
from .util import transformation_identity, algdoc, get_tiles_used
from .fine import RegistrationComputer_ICP_Point2Plane, RegistrationComputer_ICP_Point2Point

class MultiCameraBase(MultiAlignmentAlgorithm):
    """\
    Base class for multi-camera alignment algorithms.
    """
    current_pointcloud : Optional[cwipc_wrapper]
    current_pointcloud_is_new : bool
    transformations : List[RegistrationTransformation]
    original_transformations : List[RegistrationTransformation]
    results : Optional[AnalysisResults]
    analyzer_class : Optional[AnalysisAlgorithmFactory]
    analyzer : Optional[AnalysisAlgorithm]
    aligner_class : Optional[AlignmentAlgorithmFactory]
    aligner : Optional[AlignmentAlgorithm]
    verbose : bool
    show_plot : bool
    nCamera : int

    def __init__(self):
        self.current_pointcloud = None
        self.current_pointcloud_is_new = False
        self.transformations  = []
        self.original_transformations = []
        self.results = None

        self.analyzer_class = None
        self.analyzer = None
        self.aligner_class = None
        self.aligner = None

        self.verbose = False
        self.show_plot = False
        self.nCamera = 0


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
        if self.nCamera == 0:
            if self.analyzer:
                self.nCamera = self.camera_count()
            elif self.current_pointcloud:
                self.nCamera = len(get_tiles_used(self.current_pointcloud))
            else:
                assert False, "No pointcloud and no analyzer"
        return self.nCamera
    
    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        assert self.analyzer
        return self.analyzer.tilenum_for_camera_index(cam_index)

    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        assert self.analyzer
        return self.analyzer.camera_index_for_tilenum(tilenum)
    
    def _prepare_analyze(self):
        self.analyzer = None
        assert self.analyzer_class
        if self.verbose:
            print(f"{self.__class__.__name__}: Use analyzer class {self.analyzer_class.__name__}")
        self.analyzer = self.analyzer_class()
        self.analyzer.verbose = self.verbose
        assert self.current_pointcloud
        self.analyzer.add_tiled_pointcloud(self.current_pointcloud)

    def _prepare_compute(self):
        self.aligner = None
        if not self.aligner_class:
            self.aligner_class = RegistrationComputer_ICP_Point2Point
        if self.verbose:
            print(f"{self.__class__.__name__}: Use aligner class {self.aligner_class.__name__}")
        self.aligner = self.aligner_class()
        self.aligner.verbose = self.verbose
        assert self.current_pointcloud
        self.aligner.add_tiled_pointcloud(self.current_pointcloud)

    def set_original_transform(self, cam_index : int, matrix : RegistrationTransformation) -> None:
        assert self.current_pointcloud
        nCamera = get_tiles_used(self.current_pointcloud)
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
    """\
    No-op algorithm for testing purposes.
    """
    def __init__(self):
        super().__init__()

    def _prepare_compute(self):
        self.aligner = None
        assert self.current_pointcloud

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.current_pointcloud
        self._prepare_analyze()
        assert self.analyzer
        assert self.camera_count() > 0
        self.analyzer.run_twice()
        self.results = self.analyzer.get_results()
        self.results.sort_by_weight(weightstyle='order')
        if self.verbose:
            self.results.print_correspondences(label=f"{self.__class__.__name__}: Before:  Per-camera correspondence")
        if self.show_plot:
            self.analyzer.plot_label = f"{self.__class__.__name__}"
            self.analyzer.plot(show=True)
        
        return True

class MultiCameraOneToAllOthers(MultiCameraBase):
    """\
    Align multiple cameras. Every step, one camera is aligned to all others.
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
    
    def run(self) -> bool:
        """Run the algorithm"""
        assert self.current_pointcloud
        self._prepare_analyze()
        assert self.analyzer
        assert self.camera_count() > 0
        # Initialize matrices, if not done already (by our caller)
        if len(self.transformations) == 0:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())
        self.original_transformations = copy.deepcopy(self.transformations)
        # Run the analyzer for the first time, on the original pointclouds.
        self.analyzer.run_twice()
        self.results = self.analyzer.get_results()
        self.results.sort_by_weight(weightstyle='priority2')
        if self.verbose:
            self.results.print_correspondences(label=f"{self.__class__.__name__}: Before:  Per-camera correspondence, ordered by priority")
        if self.show_plot:
            self.analyzer.plot_label = f"{self.__class__.__name__}"
            self.analyzer.plot(show=True)

        # xxxjack 
        
        stepnum = 1
        camnums_already_fixed = []
        while camnum_to_fix != None:
            if self.verbose:
                print(f"{self.__class__.__name__}: Step {stepnum}: camera {camnum_to_fix}, correspondence error {correspondence}, overall correspondence error {total_correspondence}")
            # Prepare the registration computer
            self._prepare_compute()
            assert self.aligner
            correspondence = self.results.secondCorrespondence[camnum_to_fix]
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
            self.results = self.analyzer.get_results()
            self.results.sort_by_weight(weightstyle='priority')
            if self.verbose:
                self.results.print_correspondences(label=f"{self.__class__.__name__}: Step {stepnum}:  Per-camera correspondence, ordered by priority")
            if self.show_plot:
                self.analyzer.plot_label = f"{self.__class__.__name__}"
                self.analyzer.plot(show=True)

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
                    print(f"{self.__class__.__name__}: Step {stepnum}: Giving up: went only from {old_correspondence} to {correspondence}")
                break
            stepnum += 1
        self.proposed_cellsize = total_correspondence*self.cellsize_factor
        if self.verbose:
            self.results.print_correspondences(label=f"{self.__class__.__name__}: After {stepnum} steps: overall correspondence error {total_correspondence}. Per-camera correspondence, ordered worst-first")
        if self.show_plot:
            self.analyzer.plot_label = f"{self.__class__.__name__}: Step {stepnum} result"
            self.analyzer.plot(show=True)
        self._compute_change()
        if self.verbose:
            print(f"{self.__class__.__name__}: Change in matrices after alignment:")
            for cam_index in range(len(self.change)):
                print(f"\tcamindex={cam_index}, change={self.change[cam_index]}")
        self._compute_new_tiles()
        return True

    def _get_next_candidate(self, camnums_already_fixed : List[int]) -> Tuple[Optional[int], float, float]:
        assert 0
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
            print(f"{self.__class__.__name__}: Voxelizing with {self.proposed_cellsize}: point count {pc_new.count()}, was {pc.count()}")
        pointcounts = []
        for i in range(2**ntiles_orig):
            pc_tile = cwipc_tilefilter(pc_new, i)
            pointcount = pc_tile.count()
            pc_tile.free()
            pointcounts.append(pointcount)
        if self.verbose:
            print(f"{self.__class__.__name__}: Pointcounts per tile, after voxelizing:")
            for i in range(len(pointcounts)):
                print(f"\ttile {i}: {pointcounts[i]}")
        if must_free_new:
            pc_new.free()
    

class MultiCameraIterative(MultiCameraBase):
    """\
    Align multiple cameras. The first step we pick the camera with the best overal to all others.
    We move this to the destination set.

    Next we pick a camera with the best overlap with the destination set and align it to the destination set.
    We repeat this until all cameras are aligned.
    """
    resultant_pointcloud : Optional[cwipc_wrapper]
    still_to_do : List[int]
    cellsize_factor : float
    proposed_cellsize : float
    change : List[float]
    floor_correspondence : float
    two_step_correspondence : bool

    def __init__(self):
        super().__init__()
        self.resultant_pointcloud = None
        self.still_to_do = []
        self.cellsize_factor = math.sqrt(2)
        self.proposed_cellsize = 0
        self.change = []
        self.floor_correspondence = 0
        self.two_step_correspondence = False

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.current_pointcloud
        self._prepare_analyze()
        assert self.analyzer
        assert self.camera_count() > 0
        # Initialize matrices, if not done already (by our caller)
        if len(self.transformations) == 0:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())
        self.original_transformations = copy.deepcopy(self.transformations)
        
        if self.two_step_correspondence:
            self.analyzer.run_twice()
        else:
            self.analyzer.run()
        self.results = self.analyzer.get_results()
        self.results.sort_by_weight(weightstyle='bestorder')
        if self.verbose:
            self.results.print_correspondences(label=f"{self.__class__.__name__}: Before:  Per-camera correspondence")
        if self.show_plot:
            self.analyzer.plot_label = f"{self.__class__.__name__}: Before"
            self.analyzer.plot(show=True)
        self.still_to_do = list(range(self.camera_count()))
        if False: # self.two_step_correspondence:
            # Now filter out the points that we matched in the previous analysis run. The idea is that this gives
            # us a much better value for correspondence, as it will also take into account the points that are _not_
            # part of the floor.
            self.analyzer.filter_sources()
            self.analyzer.run()
            second_results = self.analyzer.get_ordered_results(weightstyle='order')
            if self.verbose:
                print(f"{self.__class__.__name__}: After filter:  Per-camera correspondence, ordered best-first:")
                for _camnum, _correspondence, _weight in second_results:
                    print(f"\tcamnum={_camnum}, correspondence={_correspondence}, weight={_weight}")
            if self.show_plot:
                self.analyzer.plot_label = f"{self.__class__.__name__}: After filter"
                self.analyzer.plot(show=True)
            self.still_to_do = second_results
            
        self._select_first_pointcloud()
        while self.still_to_do:
            assert self.resultant_pointcloud
            camnum_to_fix, correspondence = self._select_next_pointcloud_index()
            if not self.two_step_correspondence and self.floor_correspondence > 0:
                correspondence = self.floor_correspondence
                if self.verbose:
                    print(f"{self.__class__.__name__}: Set correspondence to floor correspondence {self.floor_correspondence}")
            if self.verbose:
                print(f"{self.__class__.__name__}: Aligning camera {camnum_to_fix}, corr={correspondence}, {self.resultant_pointcloud.count()} points in reference set")
            # Prepare the registration computer
            self._prepare_compute()
            assert self.aligner
            assert self.resultant_pointcloud
            if correspondence > 0:
                if self.verbose:
                    print(f"{self.__class__.__name__}: Set correspondence to {correspondence}")
                self.aligner.set_correspondence(correspondence)
            self.aligner.set_reference_pointcloud(self.resultant_pointcloud)
            self.aligner.run(camnum_to_fix)
            # Save resultant pointcloud
            self._compute_result_pointcloud_after_step()
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
        self.analyzer.run_twice()
        self.results = self.analyzer.get_results()
        max_correspondence = max(self.results.minCorrespondence)
        self.proposed_cellsize = max_correspondence * self.cellsize_factor

        if self.verbose:
            self.results.print_correspondences(label=f"{self.__class__.__name__}: After all cameras done. Per-camera correspondence")
        if self.show_plot:
            self.analyzer.plot_label = f"{self.__class__.__name__}: Final result"
            self.analyzer.plot(show=True)
        self._compute_change()
        if self.verbose:
            print(f"{self.__class__.__name__}: Change in matrices after alignment:")
            for cam_index in range(len(self.change)):
                print(f"\tcamindex={cam_index}, change={self.change[cam_index]}")
        self._compute_new_tiles()
        return True

    def _compute_result_pointcloud_after_step(self)->None:
        assert self.aligner
        assert self.resultant_pointcloud
        old_pc = self.resultant_pointcloud
        new_pc = self.aligner.get_result_pointcloud_full()
        old_pc.free()
        self.resultant_pointcloud = new_pc

    def _select_first_pointcloud(self) -> None:
        assert self.resultant_pointcloud == None
        camIndex = self.still_to_do[0]
        camNum = self.tilenum_for_camera_index(camIndex)
        self.still_to_do = self.still_to_do[1:]
        if self.verbose:
            print(f"{self.__class__.__name__}: Select initial pointcloud: camera {camNum}")
        pc = self.get_pointcloud_for_tilenum(camNum)
        # Do a deep-copy
        self.resultant_pointcloud = cwipc_from_packet(pc.get_packet())

    def _select_next_pointcloud_index(self) -> Tuple[int, float]:
        assert self.results
        camIndex = self.still_to_do[0]
        self.still_to_do = self.still_to_do[1:]
        camNum = self.results.tileNums[camIndex]
        correspondence = self.results.secondCorrespondence[camIndex]
        if self.verbose:
            print(f"{self.__class__.__name__}: Select next pointcloud: camera {camNum}, correspondence {correspondence}")
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
            # we take four points (feet, head, hand left, hand forward) and compute the average move
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
            print(f"{self.__class__.__name__}: Voxelizing with {self.proposed_cellsize}: point count {pc_new.count()}, was {pc.count()}")
        pointcounts = []
        for i in range(2**ntiles_orig):
            pc_tile = cwipc_tilefilter(pc_new, i)
            pointcount = pc_tile.count()
            pc_tile.free()
            pointcounts.append(pointcount)
        if self.verbose:
            print(f"{self.__class__.__name__}: Pointcounts per tile, after voxelizing:")
            for i in range(len(pointcounts)):
                print(f"\ttile {i}: {pointcounts[i]}")
        if must_free_new:
            pc_new.free()
    
class MultiCameraIterativeFloor(MultiCameraIterative):
    """\
    Align multiple cameras. The first step we create a point cloud with all points projected onto the plane Y=0.
    We move this to the destination set.

    Next we pick a camera with the best overlap with the destination set and align it to the destination set.
    Then we add it to the destination step.
    We repeat this until all cameras are aligned.
    """

    def _select_first_pointcloud(self) -> None:
        assert self.resultant_pointcloud == None
        assert self.current_pointcloud
        np_matrix = self.current_pointcloud.get_numpy_matrix()
        # Try and find the floor level
        point_heights = np_matrix[:,1]
        filter = point_heights < 0.1 # It's the floor so it shouldn't be off my more than 10 cm
        filtered_point_heights = point_heights[filter]
        floor_height = np.mean(filtered_point_heights)
        self.floor_correspondence = float(floor_height)
        if self.verbose:
            print(f"{self.__class__.__name__}: Floor level is {floor_height}, based on {len(filtered_point_heights)} (of {len(point_heights)}) points")
        floor_matrix = np_matrix[filter]
        floor_matrix[...,1] = 0
        self.resultant_pointcloud = cwipc_from_numpy_matrix(floor_matrix, self.current_pointcloud.timestamp())

class MultiCameraIterativeFloorOnly(MultiCameraIterativeFloor):
    """\
    Like MultiCameraIterativeFloor, but only align to the floor. I.e. don't update the destination
    set after each step.
    """
    def _compute_result_pointcloud_after_step(self) -> None:
        pass
    
class MultiCameraIterativeFloorTwice(MultiCameraIterativeFloor):
    """\
    Align multiple cameras. The first step we create a point cloud with all points projected onto the plane Y=0.
    We move this to the destination set.

    Next we pick a camera with the best overlap with the destination set and align it to the destination set.
    We repeat this until all cameras are aligned.

    After that we repeat the whole procedure, but this time with the floor created not by projecting all points but
    but by filtering out points that are too far from the floor.
    """

    def __init__(self):
        super().__init__()
        self.pass_number = 0

    def run(self) -> bool:
        self.pass_number = 1
        ok = MultiCameraIterativeFloor.run(self)
        if not ok:
            return False
        if self.verbose:
            print(f"{self.__class__.__name__}: First pass done")
        self.current_pointcloud = self.resultant_pointcloud
        
        self.resultant_pointcloud = None
        self.pass_number = 2
        self.two_step_correspondence = True
        ok = MultiCameraIterativeFloor.run(self)
        return ok
    
    def _select_first_pointcloud(self) -> None:
        MultiCameraIterativeFloor._select_first_pointcloud(self)
        if self.pass_number > 1:
            if self.verbose:
                print(f"{self.__class__.__name__}: Pass {self.pass_number}: ignore floor correspondence of {self.floor_correspondence}")
            self.floor_correspondence = 0



DEFAULT_MULTICAMERA_ALGORITHM = MultiCameraIterativeFloorTwice

ALL_MULTICAMERA_ALGORITHMS = [
    MultiCameraNoOp,
    MultiCameraOneToAllOthers,
    MultiCameraIterative,
    MultiCameraIterativeFloor,
    MultiCameraIterativeFloorOnly,
    MultiCameraIterativeFloorTwice
]


HELP_MULTICAMERA_ALGORITHMS = """
The multicamera algorithm tries to align multiple cameras to each other. It uses an alignment
algorithm repeatedly, and an analysis algorithm to determine the effect of an alignment.

The various multicamera algorithms differ in the way they select the cameras to align, and
what to try and align it to (either all other cameras, or all cameras that have been previously aligned)

The following multicamera algorithms are available:

""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_MULTICAMERA_ALGORITHMS])