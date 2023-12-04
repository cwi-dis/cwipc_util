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
    def run(self, target: Optional[int]=None) -> None:
        ...
