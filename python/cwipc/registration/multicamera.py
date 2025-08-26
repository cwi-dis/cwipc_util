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
from .util import transformation_identity, algdoc, get_tiles_used, BaseMulticamAlgorithm, cwipc_center, show_pointcloud, cwipc_colorized_copy
from .fine import RegistrationComputer_ICP_Point2Plane, DEFAULT_FINE_ALIGNMENT_ALGORITHM
from .analyze import RegistrationAnalyzer
from .plot import Plotter

OrderedCameraList = List[Tuple[int, int, float, float]] # Cameranumber, tilemask, correspondence, belowcorrespondencefraction
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
        self.aligner_class = DEFAULT_FINE_ALIGNMENT_ALGORITHM

        self.is_interactive = False
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
            self.aligner_class = DEFAULT_FINE_ALIGNMENT_ALGORITHM
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
        # Initialize matrices, if not done already (by our caller calling set_original_transform)
        if len(self.transformations) == 0:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())
        self.original_transformations = copy.deepcopy(self.transformations)

    def _pre_analyse(self, toSelf=False, toReference : Optional[cwipc_wrapper] = None, ignoreFloor : bool = False, sortBy : str='corr') -> None:
        """
        Pre-analyze the pointclouds and returns a list of camera indices in order of best to worst correspondence.
        If toSelf is true the internal nerest0point distances are computed (a measure of the quality of the capture of this camera).
        If toReference is passed in this is used as the ground truth, otherwise every camera is compared to all other cameras combined.
        If ignoreFloor is true any points with Y<0.1 are ignored.
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
                analyzer.set_correspondence_measure('mode')
                label = "toreference(mode)"
            elif toSelf:
                analyzer.set_reference_pointcloud(self.original_pointcloud, tilemask)
                analyzer.set_ignore_nearest(1) # xxxjack may want to experiment with larger values.
                analyzer.set_correspondence_measure('median')
                label = "precision(median)"
            else:
                analyzer.set_reference_pointcloud(self.original_pointcloud, othertilemask)
                analyzer.set_correspondence_measure('mode')
                label = "correspondence(mode)"
            if ignoreFloor:
                analyzer.set_ignore_floor(True)
            analyzer.run()
            results = analyzer.get_results()
            self.pre_analysis_results.append(results)

        if sortBy == 'corr':
            self.pre_analysis_results.sort(key=lambda r : r.minCorrespondence)
        elif sortBy == 'corrcount':
            self.pre_analysis_results.sort(key=lambda r : r.minCorrespondenceCount, reverse=True)
        elif sortBy == 'sourcecount':
            self.pre_analysis_results.sort(key=lambda r : r.sourcePointCount, reverse=True)
        else:
            assert False, f"Unknown sortBy={sortBy}"

        if self.verbose or self.is_interactive:
            self._print_correspondences(f"{self.__class__.__name__}: Before:  Per-camera capture {label}", self.pre_analysis_results)

        if self.show_plot:
            self._plot(f"{self.__class__.__name__}: Capture {label}", self.pre_analysis_results)

    def _todo_from_pre_analysis_results(self) -> OrderedCameraList:
        rv : OrderedCameraList = []
        for i in range(len(self.pre_analysis_results)):
            r = self.pre_analysis_results[i]
            rv.append((i, r.tilemask, r.minCorrespondence, r.minCorrespondenceCount/r.sourcePointCount))
        return rv
    
    @abstractmethod
    def run(self) -> bool:
        """Run the algorithm"""
        assert False
    
    def _post_analyse(self, toReference : Optional[cwipc_wrapper] = None) -> bool:
        assert self.original_pointcloud
        assert self.original_pointcloud.count() > 0
        assert self.camera_count() > 0
        self.results = []
        for camnum in range(self.camera_count()):
            tilemask = self.tilemask_for_camera_index(camnum)
            othertilemask = 0xff ^ tilemask
            analyzer = self._prepare_analyze()
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            if toReference:
                analyzer.set_reference_pointcloud(toReference)
                analyzer.set_correspondence_measure('mode')
                label = "toreference(mode)"
            else:
                analyzer.set_reference_pointcloud(self.original_pointcloud, othertilemask)
                analyzer.set_correspondence_measure('mode')
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
        if self.proposed_cellsize > 0.001:
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
  
    def _plot(self, title : str, results : List[AnalysisResults]) -> None:
        # xxxjack removing this so we may get camera order:
        # results.sort(key=lambda r: (r.minCorrespondence))
        plotter = Plotter(title=title)
        plotter.set_results(results)
        plotter.plot(show=True)
       
    def _print_correspondences(self, label: str, results : List[AnalysisResults]) -> None:
        # xxxjack better not to sort. results.sort(key=lambda r: (r.minCorrespondence))
        print(f"{label}:")
        for i in range(len(results)):
            r = results[i]
            print(f"\tcamnum={r.tilemask}, reference={r.referenceTilemask}, {r.tostr()}")
    
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
        self._pre_analyse(toSelf=False)
        todo = self._todo_from_pre_analysis_results()

        for camnum, tilemask, corr, fraction in todo:
            aligner = self._prepare_aligner()
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
        self._pre_analyse(toSelf=False, toReference=self.floor_pointcloud, sortBy='corrcount')
        todo = self._todo_from_pre_analysis_results()
        aligned : List[cwipc_wrapper] = []
        # xxxjack remember resultant point clouds, to combine later.
        for camnum, tilemask, corr, fraction in todo:
            aligner = self._prepare_aligner()
            aligner.set_source_pointcloud(self.original_pointcloud, tilemask)
            aligner.set_reference_pointcloud(self.floor_pointcloud)
            if self.correspondence is not None:
                corr = self.correspondence
            aligner.set_correspondence(corr)
            aligner.run()
            
            aligned.append(aligner.get_result_pointcloud())

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

class MultiCameraToGroundTruth(BaseMulticamAlignmentAlgorithm):
    """\
    Align multiple cameras to a ground truth which needs to be specified with set_groundtruth().
    Each camera is aligned to that ground truth with a max correspondence of `mode`.
    """
    # precision_threshold : float


    def __init__(self):
        super().__init__()
        # self.precision_threshold = 0.001 # Don't attempt to re-align better than 1mm
        self.groundtruth_pointcloud : Optional[cwipc_wrapper] = None

    def set_groundtruth(self, pc : cwipc_wrapper):
        self.groundtruth_pointcloud = pc

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.groundtruth_pointcloud
        assert self.camera_count() > 0
        self._init_transformations()
        self._pre_analyse(toSelf=False, toReference=self.groundtruth_pointcloud, sortBy='corrcount')
        todo = self._todo_from_pre_analysis_results()
        aligned : List[cwipc_wrapper] = []
        # xxxjack remember resultant point clouds, to combine later.
        for camnum, filemask, corr, fraction in todo:
            aligner = self._prepare_aligner()
            aligner.set_source_pointcloud(self.original_pointcloud, tilemask)
            aligner.set_reference_pointcloud(self.groundtruth_pointcloud)
            if self.correspondence is not None:
                corr = self.correspondence
            aligner.set_correspondence(corr)
            aligner.run()
            
            aligned.append(aligner.get_result_pointcloud())

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

        ok = self._post_analyse(toReference=self.groundtruth_pointcloud)
        return ok

class MultiCameraIterative(BaseMulticamAlignmentAlgorithm):
    """\
    Align multiple cameras. The first step we pick the camera with the best overal to all others.
    We move this to the destination set.

    Next we pick a camera with the best overlap with the destination set and align it to the destination set.
    We repeat this until all cameras are aligned.
    """
    current_step_target_pointcloud : Optional[cwipc_wrapper]
    current_step_in_pointcloud : Optional[cwipc_wrapper]
    current_step_out_pointcloud : Optional[cwipc_wrapper]
    current_step_results : List[AnalysisResults]

    def __init__(self):
        super().__init__()
        self.current_step_target_pointcloud : Optional[cwipc_wrapper] = None
        self.remaining_results : List[AnalysisResults] = []
    
    def _pre_step_analyse(self, stepnum : int) -> None:
        """
        Analyze the remaining camera pointclouds for how well they match the current result. Returns a list of camera indices in order of best to worst correspondence.
        """
        assert self.original_pointcloud
        assert self.current_step_target_pointcloud
        assert self.camera_count() > 1
        old_remaining_results = self.remaining_results
        assert old_remaining_results
        remaining_results : List[AnalysisResults] = []
        for rr in old_remaining_results:
            tilemask = rr.tilemask
            analyzer = self._prepare_analyze()
            analyzer.set_ignore_floor(True)
            analyzer.set_source_pointcloud(self.original_pointcloud, tilemask)
            analyzer.set_reference_pointcloud(self.current_step_target_pointcloud)
            analyzer.set_correspondence_measure("mean", "mode")
            analyzer.run()
            results = analyzer.get_results()
            remaining_results.append(results)

        if self.verbose or self.is_interactive:
            self._print_correspondences(f"{self.__class__.__name__}: Step {stepnum}:  Per-tile correspondence to target", remaining_results)
        self.remaining_results = remaining_results
    
    def _post_step_analyse(self) -> List[AnalysisResults]:
        """
        Analyze the alignment before and after this step.
        Will be used to judge whether the step was successful.
        """
        rv : List[AnalysisResults] = []
        assert self.original_pointcloud
        assert self.current_step_target_pointcloud
        assert self.current_step_in_pointcloud
        assert self.current_step_out_pointcloud
        rv : List[AnalysisResults] = []

        analyzer = self._prepare_analyze()
        analyzer.set_source_pointcloud(self.current_step_in_pointcloud)
        analyzer.set_reference_pointcloud(self.current_step_target_pointcloud)
        analyzer.set_ignore_floor(True)
        analyzer.set_correspondence_measure("mean", "mode", "median")
        analyzer.run()
        results = analyzer.get_results()
        results.tilemask = "before"
        rv.append(results)

        analyzer = self._prepare_analyze()
        analyzer.set_ignore_floor(True)
        analyzer.set_source_pointcloud(self.current_step_out_pointcloud)
        analyzer.set_reference_pointcloud(self.current_step_target_pointcloud)
        analyzer.set_ignore_floor(True)
        analyzer.set_correspondence_measure("mean", "mode", "median")
        analyzer.run()
        results = analyzer.get_results()
        results.tilemask = "after"
        rv.append(results)

        return rv

    def _accept_step(self) -> Tuple[bool, bool]:
        """Allows subclasses to accept the result of this step (or not) and to give up (or continue)"""
        return True, False
    
    def _done_step(self, tilemask : int) -> bool:
        """This tile has been aligned (and accepted). Remove from todo list."""
        for i in range(len(self.remaining_results)):
            if self.remaining_results[i].tilemask == tilemask:
                del self.remaining_results[i]
                return True
        assert False, f"Tilemask {tilemask} not in self.remaining_results"

    def _select_first_step(self) -> int:
        """Select first camera, to align others to. Returns tilemask. Can be overridden by subclasses"""
        return self.pre_analysis_results[0].tilemask
    
    def _select_next_step(self) -> Tuple[int, float, Optional[int]]:
        """Select next tile to align. Can be overridden by subclasses."""
        rr = self.remaining_results[0]
        rv = (rr.tilemask, rr.minCorrespondence, None)
        return rv
    
    def _still_to_do(self) -> List[int]:
        """Return list of tilemasks that still need to be aligned"""
        rv = list([rr.tilemask for rr in self.remaining_results])
        return rv
    
    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        assert self.camera_count() > 0
        self._init_transformations()
        self._pre_analyse(toSelf=True, ignoreFloor=True, sortBy='sourcecount')

        # The first point cloud we keep as-is, and use it as the destination set.
        first_tilemask = self._select_first_step()
        self.remaining_results = copy.copy(self.pre_analysis_results)
        self._done_step(first_tilemask)
        if self.verbose:
            print(f"{self.__class__.__name__}: First tilemask (not aligned) is {first_tilemask}")
        self.current_step_target_pointcloud = self.get_pc_for_tilemask(first_tilemask)
        step = 0
        give_up = False
        while self.remaining_results and not give_up:
            assert self.current_step_target_pointcloud
            assert self.current_step_target_pointcloud.count() > 0
            step += 1
            self._pre_step_analyse(step)
            tilemask, corr, targettile = self._select_next_step()
            if self.correspondence is not None:
                corr = self.correspondence
            if self.verbose:
                ttile = "" if targettile is None else f", targettile={targettile}"
                print(f"{self.__class__.__name__}: Step {step}: Next tilemask to align is {tilemask}. corr={corr}{ttile}")
            self.current_step_in_pointcloud = self.get_pc_for_tilemask(tilemask)
            aligner = self._prepare_aligner()
            aligner.set_source_pointcloud(self.current_step_in_pointcloud)
            aligner.set_reference_pointcloud(self.current_step_target_pointcloud, targettile)
            aligner.set_correspondence(corr)
            aligner.run()

            self.current_step_out_pointcloud = aligner.get_result_pointcloud()
            self.current_step_results = self._post_step_analyse()
            accept_step, give_up = self._accept_step()
            if accept_step:
                self._done_step(tilemask)
                new_resultant_pc = aligner.get_result_pointcloud_full()
                self.current_step_target_pointcloud.free()
                self.current_step_in_pointcloud.free()
                self.current_step_in_pointcloud = None
                self.current_step_out_pointcloud.free()
                self.current_step_out_pointcloud = None
                self.current_step_target_pointcloud = new_resultant_pc
                camnum = self.camera_index_for_tilemask(tilemask)
                # Apply new transformation (to the left of the old one)
                old_transform = self.transformations[camnum]
                this_transform = aligner.get_result_transformation()
                new_transform = np.matmul(this_transform, old_transform)
                self.transformations[camnum] = new_transform
            elif not give_up:
                self.current_step_in_pointcloud.free()
                self.current_step_in_pointcloud = None
                self.current_step_out_pointcloud.free()
                self.current_step_out_pointcloud = None

        # If we gave up there are still tiles in self.remaining_results that we have to merge into the
        # resultant full point cloud
        to_merge = self._still_to_do()
        for tilemask in to_merge:
            tile_pc = self.get_pc_for_tilemask(tilemask)
            new_pc = cwipc_join(self.current_step_target_pointcloud, tile_pc)
            self.current_step_target_pointcloud.free()
            tile_pc.free()
            self.current_step_target_pointcloud = new_pc

        assert self.current_step_target_pointcloud
        assert self.current_step_target_pointcloud.count() > 0
        self.original_pointcloud = self.current_step_target_pointcloud
        self.original_pointcloud_is_new = True
        self.current_step_target_pointcloud = None

        ok = self._post_analyse()
        return ok

class MultiCameraIterativeInteractive(MultiCameraIterative):
    """\
    Similar to MultiCameraIterative, but before every step the user can change the choices made.
    After each step the user can decide to accept it, or reject it and try something else.
    Additionally the user can decide to give up, which means keeping the registrations made so far but not
    attempting any more."""

    def __init__(self):
        super().__init__()
        self.is_interactive = True

    def _accept_step(self) -> Tuple[bool, bool]:
        while True:
            answer = self._ask("Accept this result (yes/no/giveup/show/plot)", "no default")
            if answer == "yes":
                return True, False
            if answer == "no":
                return False, False
            if answer == "giveup":
                return False, True
            if answer == "show":
                self._show_alignment()
            if answer == "plot":
                self._plot_alignment()

    def _show_alignment(self):
        assert self.current_step_in_pointcloud
        assert self.current_step_out_pointcloud
        assert self.current_step_target_pointcloud
        colored_target = cwipc_colormap(self.current_step_target_pointcloud, 0xFFFFFFFF, 0x80808080)
        colored_in = cwipc_colormap(self.current_step_in_pointcloud, 0xFFFFFFFF, 0x80AA0000)
        combined = cwipc_join(colored_target, colored_in)
        colored_target.free()
        colored_in.free()
        colored_out = cwipc_colormap(self.current_step_out_pointcloud, 0xFFFFFFFF, 0x8000AA00)
        combined2 = cwipc_join(combined, colored_out)
        combined.free()
        colored_out.free()
        show_pointcloud("Pre and Post of this step", combined2)
        combined2.free()

    def _plot_alignment(self):
        assert len(self.current_step_results) == 2
        plotter = Plotter(title="Step results")
        plotter.set_results(self.current_step_results)
        plotter.plot(show=True)

    
    def _select_first_step(self):
        tilemask = super()._select_first_step()
        pc_to_show = cwipc_colorized_copy(self.original_pointcloud)
        show_pointcloud("Captured point cloud", pc_to_show)
        tilemask = int(self._ask("Tilemask to use as reference", tilemask, options=self._still_to_do()))
        pc_to_show.free()
        return tilemask
    
    def _select_next_step(self) -> Tuple[int, float, Optional[int]]:
        tilemask, corr, ttile = super()._select_next_step()
        tilemask = int(self._ask("Tilemask to align", tilemask, options=self._still_to_do()))
        corr = float(self._ask("Max correspondence", str(corr)))
        target_tiles = get_tiles_used(self.current_step_target_pointcloud)
        if len(target_tiles) > 1:
            ttile_str = self._ask(f"Target tilemask to align to", "all", options=target_tiles)
            ttile = None if ttile_str == "all" else int(ttile_str)
        return tilemask, corr, ttile

    def _ask(self, prompt : str, default : str, options : List[Any] = []) -> str:
        option_str = ""
        if options:
            option_str_list = [str(o) for o in options]
            if not default in options:
                option_str_list.append(str(default))
            option_str = " / ".join(option_str_list)
            option_str = f"( {option_str} ) "
        sys.stdout.write(f"{prompt} {option_str}[{default}] ? ")
        sys.stdout.flush()
        line = sys.stdin.readline()
        line = line.strip()
        if not line: 
            return default
        return line
    
DEFAULT_MULTICAMERA_ALGORITHM = MultiCameraIterative

ALL_MULTICAMERA_ALGORITHMS = [
    MultiCameraOneToAllOthers,
    MultiCameraToFloor,
    MultiCameraIterative,
    MultiCameraIterativeInteractive,
    MultiCameraToGroundTruth
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