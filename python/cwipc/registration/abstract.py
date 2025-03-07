from abc import ABC, abstractmethod
from typing import Optional, Union, Any, List, Tuple, Type, Container
import math
import numpy.typing
from ..abstract import *
from .. import cwipc_wrapper

__all__ = [
    "RegistrationTransformation",
    "Algorithm",
    "AnalysisAlgorithm",
    "AnalysisResults",
    "AlignmentAlgorithm", 
    "AnalysisAlgorithmFactory",
    "AlignmentAlgorithmFactory", 
    "MultiAlignmentAlgorithm"
]

#RegistrationTransformation = numpy.typing.ArrayLike # Should be: NDArray[(4,4), float]
RegistrationTransformation = numpy.typing.NDArray[numpy.float64] # Should be: NDArray[(4,4), float]

class Algorithm(ABC):
    """Abstract base class for any algorithm that operates on tiled point clouds.
    Contains the methods for adding a pointcloud, converting from tile-index to tile-number and vv, and for running the
    algorithm.
    """
    verbose : bool
    debug : bool

    @abstractmethod
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        ...
   
    @abstractmethod
    def camera_count(self) -> int:
        """Return number of cameras (tiles) in the point clouds"""
        ...
        

    @abstractmethod
    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        ...

    @abstractmethod
    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        ...
        
    @abstractmethod
    def get_pointcloud_for_tilenum(self, tilenum : int) -> cwipc_wrapper:
        """Returns the point cloud for this tilenumber"""
        ...

    @abstractmethod
    def run(self, target: Optional[int]=None) -> bool:
        """Run the algorithm. Returns false in case of a failure."""
        ...
    # There are also methods to return the result, but they don't have a fixed signature.

class AnalysisResults:
    """Class to hold the results of an analysis algorithm"""
    #: Number of cameras (tiles) in the point clouds
    nCamera : int
    #: tile numbers for each result entry
    tileNums: List[int]
    #: minimum correspondence for each camera
    minCorrespondence : List[float]
    #: stddev for minimum correspondence
    minCorrespondenceSigma : List[float]
    #: number of points that were used for the minimum correspondence for each camera
    minCorrespondenceCount : List[int]
    #: second best correspondence for each camera
    secondCorrespondence : List[float]
    #: stddev of the above
    secondCorrespondenceSigma : List[float]
    #: number of points that were used for secondCorrespondence
    secondCorrespondenceCount : List[int]

    def __init__(self, nCamera : int):
        self.nCamera = nCamera
        self.tileNums = [0] * nCamera
        self.minCorrespondence = [0.0] * nCamera
        self.minCorrespondenceSigma = [0.0] * nCamera
        self.minCorrespondenceCount = [0] * nCamera
        self.secondCorrespondence = [0] * nCamera
        self.secondCorrespondenceSigma = [0] * nCamera
        self.secondCorrespondenceCount = [0] * nCamera
        self.overallCorrespondence = 0

    def sort_by_weight(self, weightstyle : str = 'priority') -> None:
        """Sort the results by weightstyle:
        - 'priority': the best matching camera has the heighest weight (more points give more weight, closer points give more weight)
        - 'match' : the worst matching camera has the highest weight (more points give more weight, closer points give less weight)
        - 'priority2' and 'match2': same, but for the point cloud with the first machint points filtered out
        - 'order': camera number, which is supposed to be real world order.
        """
        if weightstyle == 'priority':
            weights = [(math.log(self.minCorrespondenceCount[i]) * self.minCorrespondence[i]) for i in range(self.nCamera)] #math.log(self.matched_point_counts[camnum]) * self.correspondence[camnum]
        elif weightstyle == 'match':
            weights = [(math.log(self.minCorrespondenceCount[i]) / self.minCorrespondence[i]) for i in range(self.nCamera)]  # weight = math.log(self.matched_point_counts[camnum]) / self.correspondence[camnum]
        elif weightstyle == 'priority2':
            weights = [(math.log(self.secondCorrespondenceCount[i]) * self.secondCorrespondence[i]) for i in range(self.nCamera)] #math.log(self.matched_point_counts[camnum]) * self.correspondence[camnum]
        elif weightstyle == 'match2':
            weights = [(math.log(self.secondCorrespondenceCount[i]) / self.secondCorrespondence[i]) for i in range(self.nCamera)]  # weight = math.log(self.matched_point_counts[camnum]) / self.correspondence[camnum]
        elif weightstyle == 'order':
            weights = [camnum for camnum in range(self.nCamera)]
        else:
            assert False, f"sort_by_weight: unknown weightstyle {weightstyle}"
        sorted_indices = [i[0] for i in sorted(enumerate(weights), key=lambda x:x[1])]
        self.tileNums = [self.tileNums[i] for i in sorted_indices]
        self.minCorrespondence = [self.minCorrespondence[i] for i in sorted_indices]
        self.minCorrespondenceCount = [self.minCorrespondenceCount[i] for i in sorted_indices]
        self.secondCorrespondence = [self.secondCorrespondence[i] for i in sorted_indices]
        self.secondCorrespondenceCount = [self.secondCorrespondenceCount[i] for i in sorted_indices]

    def print_correspondences(self, label : str) -> None:
        print(f"{label}:")
        for i in range(self.nCamera):
            _camnum = self.tileNums[i]
            _minCorr = self.minCorrespondence[i]
            _minCorrCount = self.minCorrespondenceCount[i]
            _secondCorr = self.secondCorrespondence[i]
            _secondCorrCount = self.secondCorrespondenceCount[i]
            print(f"\tcamnum={_camnum}, corr={_minCorr} ({_minCorrCount} pts), corr2={_secondCorr} ({_secondCorrCount} pts)")

class AnalysisAlgorithm(Algorithm):
    """ABC for a pointcloud analysis algorithm (such as computing the overlap between tiles)"""

    plot_label : Optional[str]

    @abstractmethod
    def get_results(self) -> AnalysisResults:
        """Returns an object indicating how each camera is aligned.
        """
        ...

    @abstractmethod
    def run_twice(self) -> bool:
        """Run the algorithm twice, removing the points matched in the first run before the second run.
        This will set the secondCoeespondence variables in the result.
        """
        ...

    @abstractmethod
    def plot(self, filename : Optional[str]=None, show : bool = False, which : Optional[Container[str]]=None) -> None:
        """
        Plot the analysis results. 
        Style can be a selection of 'count', 'cumulative', 'delta' or 'all'. 'log' can be added for logarithmic scales.
        If filename is given the plot is saved there.
        If show is true the plot is shown on screen
        """
        ...
        
    @abstractmethod
    def filter_sources(self) -> None:
        """
        After running the algorithm, filter all source point clouds to remove the points that were used in the alignment.
        We can then run the algorithm again to try and determine whether there is another cluster of points with a higher correspondence.
        """

class AlignmentAlgorithm(Algorithm):
    """ABC for an algorithm that tries to find the best alignment for one tile (or possibly between two tiles, but always returning a new
    matrix for a single tile only)"""

    @abstractmethod
    def set_correspondence(self, correspondence) -> None:
        """Set the correspondence: the maximum distance between two points that are candidates for being "the same" point."""
        ...

    @abstractmethod
    def set_reference_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Set the reference point cloud to align with"""
        ...
        
    @abstractmethod
    def get_result_transformation(self) -> RegistrationTransformation:
        """After a successful run(), returns the transformation applied to the tile-under-test"""
        ...
    
    def get_result_pointcloud(self) -> cwipc_wrapper:
        """After a successful run(), returns the point cloud for the tile-under-test after the transformation has been applied"""
        ...
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
         """After a successful run(), returns the point cloud for all tiles combined, after applying transformations"""
         ...

AnalysisAlgorithmFactory = Type[AnalysisAlgorithm]
AlignmentAlgorithmFactory = Type[AlignmentAlgorithm]

class MultiAlignmentAlgorithm(Algorithm):
    """ABC for an algorithm that tries to align all tiles."""
    analyzer_class : AnalysisAlgorithmFactory
    aligner_class : AlignmentAlgorithmFactory

    def set_analyzer_class(self, analyzer_class : AnalysisAlgorithmFactory) -> None:
        """Set the class to be used for analyzing the results"""
        self.analyzer_class = analyzer_class

    def set_aligner_class(self, aligner_class : AlignmentAlgorithmFactory) -> None:
        """Set the class to be used for aligning individual tiles"""
        self.aligner_class = aligner_class
    
    @abstractmethod
    def get_result_transformations(self) -> List[RegistrationTransformation]:
        """After a successful run(), returns the list of transformations applied to each tile"""
        ...
    
    @abstractmethod
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
         """After a successful run(), returns the point cloud for all tiles combined, after applying transformations"""
         ...
