import copy
import json

DEFAULT_FILENAME="cameraconfig.json"

class CameraConfig:

    def __init__(self, confFilename, read=True):
        self.confFilename = confFilename
        self.serials = []
        self.matrices = []
        self.tree = None
        if read:
            self._readConf(self.confFilename)
            self._parseConf()
        
    def _readConf(self, confFilename):
        self.tree = json.load(open(confFilename))

    def loadConf(self, confString):
        self.tree = json.loads(confString)
        self._parseConf()
        
    def copyFrom(self, other):
        self.tree = copy.deepcopy(other.tree)
        self._parseConf()
    
    def _setcameratype(self, type):
        if 'type' in self.tree:
            assert self.tree['type'] in ('', type)
        self.tree['type'] = type
        
    def _parseConf(self):
        self.serials = []
        self.matrices = []
        for camElt in self.tree['camera']:
            self._setcameratype(camElt['type'])
            serial = camElt['serial']
            assert serial
            trafo = camElt['trafo']
            self.serials.append(serial)
            self.matrices.append(trafo)
        
    def save(self):
        json.dump(self.tree, open(self.confFilename, 'w'), indent=2)
        
    def getcount(self):
        return len(self.serials)
        
    def getserials(self):
        return self.serials
        
    def getmatrix(self, tilenum):
        return self.matrices[tilenum]
        
    def setmatrix(self, tilenum, matrix):
        self.matrices[tilenum] = copy.deepcopy(matrix)
        self.tree['camera'][tilenum]['trafo'] = self.matrices[tilenum]
   