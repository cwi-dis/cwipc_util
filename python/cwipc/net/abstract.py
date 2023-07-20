from abc import ABC, abstractmethod
from ..abstract import *

class cwipc_rawsource_abstract(ABC):

    def start(self) -> None:
        ...

    def stop(self) -> None:
        ...

    def get(self) -> bytes:
        ...

    def available(self, wait : bool=False) -> bool:
        ...

    def eof(self) -> bool:
        ...

    def statistics(self) -> None:
        ...