from abc import ABC, abstractmethod
from typing import Optional, Union, Any, List, Tuple, Type, Container
import math
import numpy.typing
from ..abstract import *
from .. import cwipc_wrapper

__all__ = [
    "RegistrationTransformation",

    "AnalysisResults",

    "OverlapAnalysisResults",
    "OverlapAnalysisAlgorithm",

    "Algorithm",
    "AnalysisAlgorithm",
    "AlignmentAlgorithm",

    "MulticamAlgorithm",
    "MulticamAlignmentAlgorithm", 

    "AnalysisAlgorithmFactory",
    "AlignmentAlgorithmFactory", 
    "MulticamAlignmentAlgorithmFactory", 
    
]

#RegistrationTransformation = numpy.typing.ArrayLike # Should be: NDArray[(4,4), float]
RegistrationTransformation = numpy.typing.NDArray[numpy.float64] # Should be: NDArray[(4,4), float]

class Algorithm(ABC):
    """Abstract base class for any algorithm that operates on two point clouds.
    Contains the methods for adding the pointclouds, running the algorithm, and returning the result.
    """
    verbose : bool
    debug : bool

    @abstractmethod
    def set_source_pointcloud(self, pc : cwipc_wrapper, tilemask : Optional[int] = None) -> None:
        """Set the source point cloud to be used during the algorithm run"""
        ...
    
    @abstractmethod
    def set_reference_pointcloud(self, pc : cwipc_wrapper, tilemask : Optional[int] = None) -> None:
        """Set the reference point cloud to be used during the algorithm run.
        """
        ...

    @abstractmethod
    def run(self) -> bool:
        """Run the algorithm. Returns false in case of a failure."""
        ...

class AnalysisResults:
    """Class to hold the results of an analysis algorithm"""
    #: minimum correspondence for each camera
    minCorrespondence : float
    #: stddev for minimum correspondence
    minCorrespondenceSigma : float
    #: number of points that were used for the minimum correspondence for each camera
    minCorrespondenceCount : int
    #: total number of points in the source point cloud
    sourcePointCount : int
    #: total number of points in the reference point cloud
    referencePointCount : int
    #: tile mask for this analysis data, if applicable
    tilemask : Optional[int]
    #: target tilemask, if applicable
    referenceTilemask : Optional[int]
    #: histogram of distances
    histogram : Optional[numpy.typing.NDArray[numpy.float64]]
    #: edges of the histogram
    histogramEdges : Optional[numpy.typing.NDArray[numpy.float64]]

class AnalysisAlgorithm(Algorithm):
    """ABC for a pointcloud analysis algorithm between two point clouds which returns a minimum distance histogram and values"""

    plot_label : Optional[str]
    correspondence_method: Optional[str]

    @abstractmethod
    def set_correspondence_method(self, method : Optional[str]):
        """Set the algorithm used to comput point cloud correspondence based on point distances.
        Values are mean, median, or mode."""
        ...
        
    @abstractmethod
    def set_max_correspondence_distance(self, correspondence : float) -> None:
        """Set the max correspondence: the maximum distance between two points that are candidates for being "the same" point."""
        ...

    @abstractmethod
    def set_min_correspondence_distance(self, correspondence : float) -> None:
        """Set the min correspondence: the smallest point distance that is meaningful. This value may be used to calculate the
        histogram to be used for the mode algorithm"""
        ...

    @abstractmethod
    def get_results(self) -> AnalysisResults:
        """Returns an object indicating how the source point cloud is aligned to the reference point cloud.
        """
        ...

class OverlapAnalysisResults:
    #: overlapping area (# of inlier correspondences / # points in the source). Higher is better.
    fitness : float
    #: RMSE of all inlier correspondences. Lower is better.
    rmse : float
    #: total number of points in the source point cloud
    sourcePointCount : int
    #: total number of points in the reference point cloud
    referencePointCount : int
    #: tile mask for this analysis data, if applicable
    tilemask : Optional[int]
    #: target tilemask, if applicable
    referenceTilemask : Optional[int]
    
class OverlapAnalysisAlgorithm(Algorithm):
    """ABC for a pointcloud analysis algorithm between two point clouds which returns an overlap indication"""

    @abstractmethod
    def set_correspondence(self, correspondence : float) -> None:
        """Set the correspondence: the maximum distance between two points that are candidates for being "the same" point."""
        ...

    @abstractmethod
    def get_results(self) -> OverlapAnalysisResults:
        """Returns an object indicating how well the two point clouds overlap
        """
        ...

AnalysisAlgorithmFactory = Type[AnalysisAlgorithm]

class AlignmentAlgorithm(Algorithm): # xxxjack wrong base class
    """ABC for an algorithm that tries to find the best alignment for one tile (or possibly between two tiles, but always returning a new
    matrix for a single tile only)"""

    @abstractmethod
    def set_correspondence(self, correspondence : float) -> None:
        """Set the correspondence: the maximum distance between two points that are candidates for being "the same" point."""
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

AlignmentAlgorithmFactory = Type[AlignmentAlgorithm]

class MulticamAlgorithm(ABC):
    """Abstract base class for any algorithm that operates on tiled point clouds.
    Contains the methods for adding a pointcloud, converting from tile-index to tile-number and vv, and for running the
    algorithm.
    """
    verbose : bool
    debug : bool

    @abstractmethod
    def set_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        ...
   
    @abstractmethod
    def camera_count(self) -> int:
        """Return number of cameras (tiles) in the point clouds"""
        ...
        
    @abstractmethod
    def tilemask_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        ...

    @abstractmethod
    def camera_index_for_tilemask(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        ...
        
#    @abstractmethod
#    def get_pointcloud_for_tilemask(self, tilenum : int) -> cwipc_wrapper:
#        """Returns the point cloud for this tilenumber"""
#        ...

    @abstractmethod
    def run(self) -> bool:
        """Run the algorithm. Returns false in case of a failure."""
        ...
    # There are also methods to return the result, but they don't have a fixed signature.




class MulticamAlignmentAlgorithm(MulticamAlgorithm):
    """ABC for an algorithm that tries to align all tiles."""
    analyzer_class : Optional[AnalysisAlgorithmFactory]
    aligner_class : Optional[AlignmentAlgorithmFactory]

    def __init__(self):
        self.analyzer_class = None
        self.aligner_class = None

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

MulticamAlignmentAlgorithmFactory = Type[MulticamAlignmentAlgorithm]
