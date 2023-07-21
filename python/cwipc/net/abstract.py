from abc import ABC, abstractmethod
from typing import Union
from ..abstract import *

# 4CC handling doesn't really belong here, but it's convenient.
vrt_fourcc_type = Union[int, bytes, str]

def VRT_4CC(code : vrt_fourcc_type) -> int:
    """Convert anything reasonable (bytes, string, int) to 4cc integer"""
    if isinstance(code, int):
        return code
    if not isinstance(code, bytes):
        assert isinstance(code, str)
        code = code.encode('ascii')
    assert len(code) == 4
    rv = (code[0]<<24) | (code[1]<<16) | (code[2]<<8) | (code[3])
    return rv

class cwipc_rawsource_abstract(ABC):

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...

    @abstractmethod
    def get(self) -> bytes:
        ...

    @abstractmethod
    def available(self, wait : bool=False) -> bool:
        ...

    @abstractmethod
    def eof(self) -> bool:
        ...

    @abstractmethod
    def statistics(self) -> None:
        ...

class cwipc_producer_abstract(ABC):

    @abstractmethod
    def is_alive(self) -> bool;
        ...

class cwipc_rawsink_abstract(ABC):

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...

    @abstractmethod
    def set_producer(self, producer : cwipc_producer_abstract) -> None:
        ...

    @abstractmethod
    def statistics(self) -> None:
        ...

