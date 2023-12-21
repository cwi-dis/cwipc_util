from abc import ABC, abstractmethod
from typing import Optional, Union, Any
from ..abstract import *
from .. import cwipc_wrapper

class RegistrationAlgorithm(ABC):

    @abstractmethod
    def add_pointcloud(self, pc : cwipc_wrapper) -> int:
        """Add a pointcloud to be used during the algorithm run. Returns "tilenum" for this pointcloud"""
        ...
    
    @abstractmethod
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
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
    def run(self, target: Optional[int]=None) -> bool:
        ...

    @abstractmethod
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        """Seve the resulting plot"""
        ...