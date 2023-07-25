from abc import ABC
from typing import List
from ...net.abstract import *

matrix_type = List[List[float]]

class PointCloudAbstract(ABC):
    pass

class CalibratorGrabberAbstract(ABC):
    
    @abstractmethod
    def open(self) -> bool:
        ...
    
    @abstractmethod
    def getcount(self) -> int:
        ...
    
    @abstractmethod
    def getserials(self) -> List[str]:
        ...
        
    def getmatrix(self, tilenum : int) -> matrix_type:
        ...
        
    def getpointcloud(self) -> PointCloudAbstract:
        ...
