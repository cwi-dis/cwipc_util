import sys
import os
import cwipc
from typing import Optional, List

from .abstract import CalibratorGrabberAbstract
from ... import cwipc_wrapper
from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig, DEFAULT_FILENAME, matrix_type

class FileGrabber(CalibratorGrabberAbstract):
    cameraconfig : Optional[CameraConfig]

    def __init__(self, plyfile : str):
        self.pcFilename = plyfile
        self.cameraconfig = None
        
    def open(self) -> bool:
        if not os.path.exists(self.pcFilename):
            print(f'File not found: {self.pcFilename}', file=sys.stderr)
            return False
        dirname = os.path.dirname(self.pcFilename)
        confFilename = os.path.join(dirname, DEFAULT_FILENAME)
        if not os.path.exists(confFilename):
            print(f'File not found: {confFilename}', file=sys.stderr)
            return False
        self.cameraconfig = CameraConfig(confFilename, read=True)
        return True
        
    def getcount(self) -> int:
        assert self.cameraconfig
        return self.cameraconfig.getcount()
        
    def getserials(self) -> List[str]:
        assert self.cameraconfig
        return self.cameraconfig.getserials()
        
    def getmatrix(self, tilenum : int) -> matrix_type:
        assert self.cameraconfig
        return self.cameraconfig.getmatrix(tilenum)
        
    def getpointcloud(self) -> Pointcloud:
        pc = cwipc.cwipc_read(self.pcFilename, 0)
        return Pointcloud.from_cwipc(pc)
