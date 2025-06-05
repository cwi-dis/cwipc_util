from abc import ABC, abstractmethod
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from cwipc import cwipc_wrapper, cwipc_from_packet, cwipc_from_numpy_matrix, cwipc_join

from cwipc.registration.abstract import RegistrationTransformation
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_downsample, cwipc_write
from .abstract import *
from .util import transformation_identity, algdoc, get_tiles_used, BaseMulticamAlgorithm
from .fine import RegistrationComputer_ICP_Point2Plane, RegistrationComputer_ICP_Point2Point
from .plot import Plotter
class BaseMulticamAlignmentAlgorithm(MulticamAlignmentAlgorithm, BaseMulticamAlgorithm):
    """\
    Base class for multi-camera alignment algorithms.
    """
    original_pointcloud : Optional[cwipc_wrapper]
    transformations : List[RegistrationTransformation]
    original_transformations : List[RegistrationTransformation]
    pre_analysis_results : List[AnalysisResults]
    results : List[AnalysisResults]
    verbose : bool
    show_plot : bool
    nCamera : int
    change : List[float]
    cellsize_factor : float
    proposed_cellsize : float

    def __init__(self):
        MulticamAlignmentAlgorithm.__init__(self)
        BaseMulticamAlgorithm.__init__(self)
        self.original_pointcloud = None
        self.transformations  = []
        self.original_transformations = []
        self.pre_analysis_results = []
        self.results = []

        self.verbose = False
        self.show_plot = False
        self.nCamera = 0

        self.change = []
        self.cellsize_factor = math.sqrt(2)
        self.proposed_cellsize = 0

    def _prepare_analyze(self) -> AnalysisAlgorithm:
        analyzer = None
        assert self.analyzer_class
        if self.verbose:
            print(f"{self.__class__.__name__}: Use analyzer class {self.analyzer_class.__name__}")
        analyzer = self.analyzer_class()
        analyzer.verbose = self.verbose
        return analyzer

    def _prepare_aligner(self) -> AlignmentAlgorithm:
        if not self.aligner_class:
            self.aligner_class = RegistrationComputer_ICP_Point2Point
        if self.verbose:
            print(f"{self.__class__.__name__}: Use aligner class {self.aligner_class.__name__}")
        aligner = self.aligner_class()
        aligner.verbose = self.verbose
        return aligner

    def set_original_transform(self, cam_index : int, matrix : RegistrationTransformation) -> None:
        assert self.original_pointcloud
        nCamera = get_tiles_used(self.original_pointcloud)
        if len(self.transformations) == 0:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())
        self.transformations[cam_index] = matrix

    def _init_transformations(self) -> None:
        # Initialize matrices, if not done already (by our caller)
        if len(self.transformations) == 0:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())
        self.original_transformations = copy.deepcopy(self.transformations)

    def _pre_analyse(self) -> List[int]:
        """
        Pre-analyze the pointclouds and returns a list of camera indices in order of best to worst correspondence.
        """
        assert self.original_pointcloud
        assert self.camera_count() > 1
        self.pre_analysis_results = []
        for camnum in range(self.camera_count()):
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            analyzer.set_reference_pointcloud(self.original_pointcloud, othertilemask)
            analyzer.run()
            results = analyzer.get_results()
            self.pre_analysis_results.append(results)

        if self.verbose:
            self._print_correspondences(f"{self.__class__.__name__}: Before:  Per-camera correspondence", self.pre_analysis_results)

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: Before", self.pre_analysis_results)

        camnums_and_correspondences = []
        for i in range(len(self.pre_analysis_results)):
            r = self.pre_analysis_results[i]
            camnums_and_correspondences.append((i, r.minCorrespondence))
        camnums_and_correspondences.sort(key=lambda x: x[1])
        todo = [camnum for camnum, _ in camnums_and_correspondences]
        return todo
    
    @abstractmethod
    def run(self) -> bool:
        """Run the algorithm"""
        assert False
    
    def _post_analyse(self) -> bool:
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self.results = []
        for camnum in range(self.camera_count()):
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            analyzer.set_reference_pointcloud(self.original_pointcloud, othertilemask)
            analyzer.run()
            results = analyzer.get_results()
            self.results.append(results)
        
        if self.verbose:
            self._print_correspondences(f"{self.__class__.__name__}: After:  Per-camera correspondence", self.results)

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: After", self.results)
        
        correspondences = [r.minCorrespondence for r in self.results]
        max_correspondence = max(correspondences)

        self.proposed_cellsize = max_correspondence*self.cellsize_factor
        self._compute_change()
        if self.verbose:
            print(f"{self.__class__.__name__}: Change in matrices after alignment:")
            for cam_index in range(len(self.change)):
                print(f"\tcamindex={cam_index}, change={self.change[cam_index]}")
        self._compute_new_tiles()
        return True

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
        assert self.original_pointcloud
        pc = self.original_pointcloud
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

    def get_result_transformations(self) -> List[RegistrationTransformation]:
        return self.transformations
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        assert self.original_pointcloud
        if not self.original_pointcloud_is_new:
            # Do a deep-copy, so our caller can free the pointcloud it passed to us
            self.original_pointcloud = cwipc_from_packet(self.original_pointcloud.get_packet())
            self.original_pointcloud_is_new = True
        return self.original_pointcloud
  
    def _plot(self, title : str, data : List[AnalysisResults]) -> None:
        data.sort(key=lambda r: (r.minCorrespondence + r.minCorrespondenceSigma))
        plotter = Plotter(title=title)
        plotter.set_results(data)
        plotter.plot(show=True)
       
    def _print_correspondences(self, label: str, data : List[AnalysisResults]) -> None:
        data.sort(key=lambda r: (r.minCorrespondence + r.minCorrespondenceSigma))
        print(f"{label}:")
        for i in range(len(data)):
            r = data[i]
            print(f"\tcamnum={r.tilemask}, reference={r.referenceTilemask}, correspondence={r.minCorrespondence}, stddev={r.minCorrespondenceSigma}, count={r.minCorrespondenceCount}")

class MultiCameraNoOp(BaseMulticamAlignmentAlgorithm):
    """\
    No-op algorithm for testing purposes. Only computes distances between each tile and all other tiles.
    """
    def __init__(self):
        super().__init__()

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.camera_count() > 1
        _ = self._pre_analyse()

        return True
    
class MultiCameraOneToAllOthers(BaseMulticamAlignmentAlgorithm):
    """\
    Align multiple cameras. Every step, one camera is aligned to all others.
    Every step, we pick the camera with the best chances to make the biggest change.
    """
    # precision_threshold : float


    def __init__(self):
        super().__init__()
        # self.precision_threshold = 0.001 # Don't attempt to re-align better than 1mm

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self._init_transformations()
        todo = self._pre_analyse()

        for camnum in todo:
            aligner = self._prepare_aligner()
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            aligner.set_source_pointcloud(self.original_pointcloud, tilemask)
            aligner.set_reference_pointcloud(self.original_pointcloud, othertilemask)
            # aligner.set_correspondence(self.precision_threshold)
            aligner.run()
            
            # Remember resultant pointcloud
            old_pc = self.original_pointcloud
            new_pc = aligner.get_result_pointcloud_full()
            old_pc.free()
            self.original_pointcloud = new_pc
            self.original_pointcloud_is_new = True

            # Apply new transformation (to the left of the old one)
            old_transform = self.transformations[camnum]
            this_transform = aligner.get_result_transformation()
            new_transform = np.matmul(this_transform, old_transform)
            self.transformations[camnum] = new_transform

        ok = self._post_analyse()
        return ok

    

class MultiCameraIterative(BaseMulticamAlignmentAlgorithm):
    """\
    Align multiple cameras. The first step we pick the camera with the best overal to all others.
    We move this to the destination set.

    Next we pick a camera with the best overlap with the destination set and align it to the destination set.
    We repeat this until all cameras are aligned.
    """
    resultant_pointcloud : Optional[cwipc_wrapper]

    def __init__(self):
        super().__init__()
        self.resultant_pointcloud = None
    
    def _get_pc_for_camnum(self, camnum: int) -> cwipc_wrapper:
        """Get the pointcloud for a given camera number"""
        assert self.original_pointcloud
        tilemask = self.tilemask_for_camera_index(camnum)
        pc = cwipc_tilefilter(self.original_pointcloud, tilemask)
        if not pc:
            raise ValueError(f"Camera {camnum} has no point cloud")
        return pc
    
    def _mid_analyse(self, remaining : List[int]) -> List[int]:
        """
        Analyze the remaining camera pointclouds for how well they match the current result. Returns a list of camera indices in order of best to worst correspondence.
        """
        assert self.original_pointcloud
        assert self.resultant_pointcloud
        assert self.camera_count() > 1
        remaining_results : List[AnalysisResults] = []
        for camnum in remaining:
            tilemask = self.tilemask_for_camera_index(camnum)
            analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            analyzer.set_reference_pointcloud(self.resultant_pointcloud)
            analyzer.run()
            results = analyzer.get_results()
            remaining_results.append(results)

        if self.verbose:
            self._print_correspondences(f"{self.__class__.__name__}: Mid:  Per-camera correspondence", remaining_results)


        camnums_and_mismatch = []
        for i in range(len(remaining_results)):
            r = remaining_results[i]
            assert r.tilemask
            camnum = self.camera_index_for_tilemask(r.tilemask)
            mismatch = (r.minCorrespondence + r.minCorrespondenceSigma) * (r.sourcePointCount - r.minCorrespondenceCount)
            camnums_and_mismatch.append((camnum, mismatch))
        camnums_and_mismatch.sort(key=lambda x: x[1])
        print(f"xxxjack: {self.__class__.__name__}: Mid:  Sorted cameras by mismatch: {camnums_and_mismatch}")
        todo = [camnum for camnum, _ in camnums_and_mismatch]

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: Mid", remaining_results)

        return todo
        
    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self._init_transformations()
        todo = self._pre_analyse()

        # The first point cloud we keep as-is, and use it as the destination set.
        first = todo.pop(0)
        if self.verbose:
            print(f"{self.__class__.__name__}: First camera to align is {first}")
        self.resultant_pointcloud = self._get_pc_for_camnum(first)
        while todo:
            if len(todo) > 1:
                todo = self._mid_analyse(todo)
            camnum = todo.pop(0)
            if self.verbose:
                print(f"{self.__class__.__name__}: Next camera to align is {camnum}")
            next_pc = self._get_pc_for_camnum(camnum)
            aligner = self._prepare_aligner()
            aligner.set_source_pointcloud(next_pc)
            aligner.set_reference_pointcloud(self.resultant_pointcloud)
            # aligner.set_correspondence(self.precision_threshold)
            aligner.run()

            new_next_pc = aligner.get_result_pointcloud()
            new_resultant_pc = cwipc_join(self.resultant_pointcloud, new_next_pc)
            self.resultant_pointcloud.free()
            next_pc.free()
            self.resultant_pointcloud = new_resultant_pc

            # Apply new transformation (to the left of the old one)
            old_transform = self.transformations[camnum]
            this_transform = aligner.get_result_transformation()
            new_transform = np.matmul(this_transform, old_transform)
            self.transformations[camnum] = new_transform

        self.original_pointcloud = self.resultant_pointcloud
        self.original_pointcloud_is_new = True
        self.resultant_pointcloud = None

        ok = self._post_analyse()
        return ok

DEFAULT_MULTICAMERA_ALGORITHM = MultiCameraIterative

ALL_MULTICAMERA_ALGORITHMS = [
    MultiCameraNoOp,
    MultiCameraOneToAllOthers,
    MultiCameraIterative,
#    MultiCameraIterativeFloor,
]


HELP_MULTICAMERA_ALGORITHMS = """
The multicamera algorithm tries to align multiple cameras to each other. It uses an alignment
algorithm repeatedly, and an analysis algorithm to determine the effect of an alignment.

The various multicamera algorithms differ in the way they select the cameras to align, and
what to try and align it to (either all other cameras, or all cameras that have been previously aligned)

The following multicamera algorithms are available:

""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_MULTICAMERA_ALGORITHMS])