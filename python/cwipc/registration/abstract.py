from abc import ABC, abstractmethod
from typing import Optional, Union, Any, List, Tuple, Type
import numpy.typing
from ..abstract import *
from .. import cwipc_wrapper

__all__ = [
    "RegistrationTransformation",
    "Algorithm",
    "AnalysisAlgorithm",
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
    def get_pointcloud_for_camera_index(self, cam_index : int) -> cwipc_wrapper:
        """Returns the point cloud for this tilenumber"""
        ...

    @abstractmethod
    def run(self, target: Optional[int]=None) -> bool:
        """Run the algorithm. Returns false in case of a failure."""
        ...
    # There are also methods to return the result, but they don't have a fixed signature.

class AnalysisAlgorithm(Algorithm):
    """ABC for a pointcloud analysis algorithm (such as computing the overlap between tiles)"""

    @abstractmethod
    def get_ordered_results(self) -> List[Tuple[int, float, float]]:
        """Returns a list of (tilenum, epsilon, weight) indicating how each camera is aligned.
        
        tilenum is the camera tile number
        epsilon is a measure (in meters) for how closely this camera tile matches the others
        weight is based on espilon and the number of points that were matched

        The list is sorted by weight (decreasing), so the expectation is that fixing the first one
        will lead to the greatest overall improvement.
        """

    @abstractmethod
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        """Seve the resulting plot"""
        ...

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
