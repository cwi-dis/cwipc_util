import sys
import os
import cwipc

from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig, DEFAULT_FILENAME

class FileGrabber:
    def __init__(self, plyfile):
        self.pcFilename = plyfile
        self.cameraconfig = None
        
    def open(self):
        if not os.path.exists(self.pcFilename):
            print(f'File not found: {self.pcFilename}', file=sys.stderr)
            return False
        dirname = os.path.dirname(self.pcFilename)
        confFilename = os.path.join(dirname, DEFAULT_FILENAME)
        if not os.path.exists(confFilename):
            print(f'File not found: {confFilename}', file=sys.stderr)
            return False
        self.cameraconfig = CameraConfig(confFilename)
        return True
        
    def getcount(self):
        return self.cameraconfig.getcount()
        
    def getserials(self):
        return self.cameraconfig.getserials()
        
    def getmatrix(self, tilenum):
        return self.cameraconfig.getmatrix(tilenum)
        
    def getpointcloud(self):
        pc = cwipc.cwipc_read(self.pcFilename, 0)
        return Pointcloud.from_cwipc(pc)
