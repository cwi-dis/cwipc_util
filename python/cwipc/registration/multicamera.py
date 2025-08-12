import sys
from abc import ABC, abstractmethod
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from cwipc import cwipc_wrapper, cwipc_from_packet, cwipc_from_numpy_matrix, cwipc_join

from cwipc.registration.abstract import RegistrationTransformation
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_downsample, cwipc_write, cwipc_colormap
from .abstract import *
from .util import transformation_identity, algdoc, get_tiles_used, BaseMulticamAlgorithm
from .fine import RegistrationComputer_ICP_Point2Plane, RegistrationComputer_ICP_Point2Point
from .analyze import RegistrationAnalyzer
from .plot import Plotter

OrderedCameraList = List[Tuple[int, float, float]] # Cameranumber, correspondence, belowcorrespondencefraction
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
    correspondence : Optional[float]

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
        self.cellsize_factor = math.sqrt(0.5) # xxxjack or 1, or math.sqrt(2)
        self.proposed_cellsize = 0
        self.correspondence = None
        
    def set_max_correspondence(self, max_correspondence: float) -> None:
        """Set the maximum correspondence for this algorithm"""
        self.correspondence = max_correspondence

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

    def _pre_analyse(self, toSelf=False, toReference : Optional[cwipc_wrapper] = None) -> OrderedCameraList:
        """
        Pre-analyze the pointclouds and returns a list of camera indices in order of best to worst correspondence.
        """
        assert self.original_pointcloud
        assert self.camera_count() > 1
        self.pre_analysis_results = []
        for camnum in range(self.camera_count()):
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            if toSelf:
                analyzer = RegistrationAnalyzer()
                analyzer.verbose = self.verbose
            else:
                analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            if toReference != None:
                analyzer.set_reference_pointcloud(toReference)
                analyzer.set_correspondence_method('mode')
                label = "toreference(mode)"
            elif toSelf:
                analyzer.set_reference_pointcloud(self.original_pointcloud, tilemask)
                analyzer.set_ignore_nearest(1) # xxxjack may want to experiment with larger values.
                analyzer.set_correspondence_method('median')
                label = "precision(median)"
            else:
                analyzer.set_reference_pointcloud(self.original_pointcloud, othertilemask)
                analyzer.set_correspondence_method('mode')
                label = "correspondence(mode)"
            analyzer.run()
            results = analyzer.get_results()
            self.pre_analysis_results.append(results)

        if self.verbose:
            self._print_correspondences(f"{self.__class__.__name__}: Before:  Per-camera capture {label}", self.pre_analysis_results)

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: Capture {label}", self.pre_analysis_results)

        camnums_and_correspondences : OrderedCameraList = []
        for i in range(len(self.pre_analysis_results)):
            r = self.pre_analysis_results[i]
            camnums_and_correspondences.append((i, r.minCorrespondence, r.minCorrespondenceCount/r.sourcePointCount))
        camnums_and_correspondences.sort(key=lambda x: x[1])
        return camnums_and_correspondences
    
    @abstractmethod
    def run(self) -> bool:
        """Run the algorithm"""
        assert False
    
    def _post_analyse(self, toReference : Optional[cwipc_wrapper] = None) -> bool:
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self.results = []
        for camnum in range(self.camera_count()):
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            if toReference:
                analyzer.set_reference_pointcloud(toReference)
                analyzer.set_correspondence_method('mode')
                label = "toreference(mode)"
            else:
                analyzer.set_reference_pointcloud(self.original_pointcloud, othertilemask)
                analyzer.set_correspondence_method('mode')
                label = "correspondence(mode)"
            analyzer.run()
            results = analyzer.get_results()
            self.results.append(results)
        
        if self.verbose:
            self._print_correspondences(f"{self.__class__.__name__}: After:  Per-camera {label}", self.results)

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: After {label}", self.results)
        
        correspondences = [r.minCorrespondence for r in self.results]
        max_correspondence = max(correspondences)
        min_correspondence = min(correspondences)
        avg_correspondence = (max_correspondence+min_correspondence)/2

        self.proposed_cellsize = min_correspondence*self.cellsize_factor
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
        todo = self._pre_analyse(toSelf=False)

        for camnum, corr, fraction in todo:
            aligner = self._prepare_aligner()
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            aligner.set_source_pointcloud(self.original_pointcloud, tilemask)
            aligner.set_reference_pointcloud(self.original_pointcloud, othertilemask)
            if self.correspondence is not None:
                corr = self.correspondence
            aligner.set_correspondence(corr)
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

class MultiCameraToFloor(BaseMulticamAlignmentAlgorithm):
    """\
    Align multiple cameras to the floor at Y=0. Requires enough floor to be visible for each camera.
    A synthetic floor is computed by projecting all points to Y=0.
    Subsequently each camera is aligned to that floor with a max correspondence of `mode`.
    """
    # precision_threshold : float


    def __init__(self):
        super().__init__()
        # self.precision_threshold = 0.001 # Don't attempt to re-align better than 1mm
        self.floor_pointcloud : Optional[cwipc_wrapper] = None

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self._init_transformations()
        self._prepare_floor()
        assert self.floor_pointcloud
        todo = self._pre_analyse(toSelf=False, toReference=self.floor_pointcloud)
        aligned : List[cwipc_wrapper] = []
        # xxxjack remember resultant point clouds, to combine later.
        for camnum, corr, fraction in todo:
            aligner = self._prepare_aligner()
            tilemask = self.tilemask_for_camera_index(camnum)
            aligner.set_source_pointcloud(self.original_pointcloud, tilemask)
            aligner.set_reference_pointcloud(self.floor_pointcloud)
            if self.correspondence is not None:
                corr = self.correspondence
            aligner.set_correspondence(corr)
            aligner.run()
            
            aligned.append(aligner.get_result_pointcloud())
            if False:
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
        result = aligned.pop()
        while aligned:
            next = aligned.pop()
            result = cwipc_join(result, next)
            next.free()
        self.original_pointcloud = result
        self.original_pointcloud_is_new = True

        ok = self._post_analyse(toReference=self.floor_pointcloud)
        return ok

    def _prepare_floor(self) -> None:
        assert self.original_pointcloud
        ndarray = self.original_pointcloud.get_numpy_matrix()
        ndarray[:,1] = 0
        self.floor_pointcloud = cwipc_from_numpy_matrix(ndarray, 0)

    

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
    
    def _mid_analyse(self, stepnum : int) -> None:
        """
        Analyze the remaining camera pointclouds for how well they match the current result. Returns a list of camera indices in order of best to worst correspondence.
        """
        assert self.original_pointcloud
        assert self.resultant_pointcloud
        assert self.camera_count() > 1
        assert self.todo
        remaining = self.todo
        remaining_results : List[AnalysisResults] = []
        for camnum, _, _ in remaining:
            tilemask = self.tilemask_for_camera_index(camnum)
            analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            analyzer.set_reference_pointcloud(self.resultant_pointcloud)
            analyzer.run()
            results = analyzer.get_results()
            remaining_results.append(results)

        if self.verbose:
            self._print_correspondences(f"{self.__class__.__name__}: Step {stepnum}:  Per-camera correspondence", remaining_results)


        camnums_and_mismatch : OrderedCameraList = []
        for i in range(len(remaining_results)):
            r = remaining_results[i]
            assert r.tilemask
            camnum = self.camera_index_for_tilemask(r.tilemask)
            if False:
                mismatch = (r.minCorrespondence + r.minCorrespondenceSigma) * (r.sourcePointCount - r.minCorrespondenceCount)
            else:
                mismatch = r.minCorrespondence
            camnums_and_mismatch.append((camnum, mismatch, r.minCorrespondenceCount / r.sourcePointCount))
        camnums_and_mismatch.sort(key=lambda x: -x[2])
        print(f"xxxjack: {self.__class__.__name__}: Step {stepnum}:  Sorted cameras by expected improvement: {camnums_and_mismatch}")

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: Before step {stepnum}", remaining_results)

        self.todo = camnums_and_mismatch

    def _accept_step(self) -> bool:
        """Allows subclasses to not accept the result of this step"""
        return True
    
    def _done(self, camnum : int) -> bool:
        """This tile has been aligned (and accepted). Remove from todo list."""
        for i in range(len(self.todo)):
            if self.todo[i][0] == camnum:
                del self.todo[i]
                return True
        assert False, f"Camnum {camnum} not in self.todo"

    def _select_first(self) -> int:
        """Select first camera, to align others to. Can be overridden by subclasses"""
        return self.todo[0][0]
    
    def _select_next(self) -> Tuple[int, float, float]:
        """Select next tile to align. Can be overridden by subclasses."""
        assert self.todo
        return self.todo[0]
    
    def dump_pointclouds(self, filename: str, source: cwipc_wrapper, target: cwipc_wrapper):
        if self.verbose:
            print(f"Dumping point clouds to {filename}")
        colored_source = cwipc_colormap(source, 0xFFFFFFFF, 0xAAFF0000)
        colored_target = cwipc_colormap(target, 0xFFFFFFFF, 0xAA00FF00)
        combined = cwipc_join(colored_source, colored_target)
        cwipc_write(filename, combined)
        colored_source.free()
        colored_target.free()
        combined.free()

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self._init_transformations()
        _ = self._pre_analyse(toSelf=False) # Only needed for verbose output and plots.
        self.todo = self._pre_analyse(toSelf=True)

        # The first point cloud we keep as-is, and use it as the destination set.
        first = self._select_first()
        self._done(first)
        if self.verbose:
            print(f"{self.__class__.__name__}: First camera (not aligned) is {first}")
        self.resultant_pointcloud = self._get_pc_for_camnum(first)
        step = 0
        while self.todo:
            step += 1
            if len(self.todo) > 1:
                self._mid_analyse(step)
            camnum, corr, fraction = self._select_next()
            if self.correspondence is not None:
                corr = self.correspondence
            if self.verbose:
                print(f"{self.__class__.__name__}: Step {step}: Next camera to align is {camnum}. corr={corr}, fraction={fraction}")
            next_pc = self._get_pc_for_camnum(camnum)
            self.dump_pointclouds(f"multicamera_iterative_step_{step}_in_cam{camnum}.ply", self.resultant_pointcloud, next_pc)
            aligner = self._prepare_aligner()
            aligner.set_source_pointcloud(next_pc)
            aligner.set_reference_pointcloud(self.resultant_pointcloud)
            aligner.set_correspondence(corr)
            aligner.run()

            new_next_pc = aligner.get_result_pointcloud()
            self.dump_pointclouds(f"multicamera_iterative_step_{step}_out_cam{camnum}.ply", self.resultant_pointcloud, new_next_pc)
            new_resultant_pc = cwipc_join(self.resultant_pointcloud, new_next_pc)
            if self._accept_step():
                self._done(camnum)
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

class MultiCameraIterativeInteractive(MultiCameraIterative):
    """\
    Similar to MultiCameraIterative, but before every step the user can change the choices made.
    After each step the user can decide to accept it, or reject it and try something else"""

    def _accept_step(self) -> bool:
        return self._ask("Accept this result (yes/no)", "no") == "yes"
    
    def _select_first(self):
        camnum = super()._select_first()
        camnum = int(self._ask("Tile index to align to", str(camnum)))
        return camnum
    
    def _select_next(self) -> Tuple[int, float, float]:
        camnum, corr, fraction = super()._select_next()
        camnum = int(self._ask("Tile index to align", str(camnum)))
        corr = float(self._ask("Max correspondence", str(corr)))
        return camnum, corr, fraction

    def _ask(self, prompt : str, default : str) -> str:
        sys.stdout.write(f"{prompt} [{default}] ? ")
        sys.stdout.flush()
        line = sys.stdin.readline()
        line = line.strip()
        if not line: 
            return default
        return line
    
DEFAULT_MULTICAMERA_ALGORITHM = MultiCameraIterative

ALL_MULTICAMERA_ALGORITHMS = [
    MultiCameraNoOp,
    MultiCameraOneToAllOthers,
    MultiCameraToFloor,
    MultiCameraIterative,
    MultiCameraIterativeInteractive,
]


HELP_MULTICAMERA_ALGORITHMS = """

## Multicamera algorithms
 
The multicamera algorithm --algorithm_multicamera tries to align multiple cameras to each 
other. It uses an alignment algorithm repeatedly, and an analysis algorithm to determine 
the effect of an alignment.

The various multicamera algorithms differ in the way they select the cameras to align, and
what to try and align it to (either all other cameras, or all cameras that have been 
previously aligned).

Default multicamera algorithm is """ + DEFAULT_MULTICAMERA_ALGORITHM.__name__ + """.

The following multicamera algorithms are available:

""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_MULTICAMERA_ALGORITHMS])