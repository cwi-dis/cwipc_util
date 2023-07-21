from abc import ABC, abstractmethod
from typing import Optional

class cwipc_abstract(ABC):
    
    @abstractmethod
    def free(self) -> None:
        """Delete the opaque pointcloud object (by asking the original creator to do so)"""
        ...
        
    @abstractmethod
    def timestamp(self) -> int:
        """Returns timestamp (microseconds) when this pointcloud was captured (relative to some unspecified origin)"""
        ...
        
    @abstractmethod
    def cellsize(self) -> float:
        """Returns the size of the cells this pointcloud represents (0 if unknown)"""
        ...
        
    @abstractmethod
    def count(self) -> int:
        """Get the number of points in the pointcloud"""
        ...
        
    
class cwipc_source_abstract(ABC):
    @abstractmethod
    def free(self) -> None:
        """Delete the opaque pointcloud source object (by asking the original creator to do so)"""
        ...
        
    @abstractmethod
    def eof(self) -> bool:
        """Return True if no more pointclouds will be forthcoming"""
        ...
        
    @abstractmethod
    def available(self, wait : bool) -> bool:
        """Return True if a pointcloud is currently available. The wait parameter signals the source may wait a while."""
        ...

    @abstractmethod
    def get(self) -> Optional[cwipc_abstract]:
        """Get a cwipc (opaque pointcloud) from this source. Returns None if no more pointcloudes are forthcoming"""
        ...

    @abstractmethod
    def request_auxiliary_data(self, name : str) -> None:
        """Ask this grabber to also provide auxiliary data `name` with each pointcloud"""
        ...

    @abstractmethod
    def auxiliary_data_requested(self, name : str) -> bool:
        """Return True if this grabber provides auxiliary data `name` with each pointcloud"""
        ...

class cwipc_sink_abstract(ABC):
    pass