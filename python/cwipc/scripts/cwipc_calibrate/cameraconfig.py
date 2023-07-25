import copy
import json
from typing import Any, List, Optional, Union

from .abstract import matrix_type

DEFAULT_FILENAME="cameraconfig.json"


class CameraConfig:
    tree : Optional[dict[str, Any]]
    serials : List[str]

    def __init__(self, confFilename : str, read=True):
        self.confFilename = confFilename
        self.serials = []
        self.matrices = []
        self.tree = None
        if read:
            self._readConf(self.confFilename)
            self._parseConf()
        
    def _readConf(self, confFilename : str) -> None:
        self.tree = json.load(open(confFilename))

    def loadConf(self, confString : Union[str, bytes]) -> None:
        self.tree = json.loads(confString)
        self._parseConf()
        
    def copyFrom(self, other : 'CameraConfig') -> None:
        self.tree = copy.deepcopy(other.tree)
        self._parseConf()
    
    def _setcameratype(self, type : str) -> None:
        assert self.tree
        if 'type' in self.tree:
            assert self.tree['type'] in ('', type)
        self.tree['type'] = type
        
    def _parseConf(self) -> None:
        assert self.tree
        self.serials = []
        self.matrices = []
        for camElt in self.tree['camera']:
            self._setcameratype(camElt['type'])
            serial = camElt['serial']
            assert serial
            trafo = camElt['trafo']
            self.serials.append(serial)
            self.matrices.append(trafo)
        
    def save(self) -> None:
        json.dump(self.tree, open(self.confFilename, 'w'), indent=2)
        
    def getcount(self) -> int:
        return len(self.serials)
        
    def getserials(self) -> List[str]:
        return self.serials
        
    def getmatrix(self, tilenum : int) -> matrix_type:
        return self.matrices[tilenum]
        
    def setmatrix(self, tilenum : int, matrix : matrix_type) -> None:
        assert self.tree
        self.matrices[tilenum] = copy.deepcopy(matrix)
        self.tree['camera'][tilenum]['trafo'] = self.matrices[tilenum]
   