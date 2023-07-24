import sys
import os
import os.path
from typing import Optional, cast

from ... import CwipcError, cwipc_wrapper
from ...net.abstract import cwipc_source_factory_abstract, cwipc_tiledsource_abstract
from .cameraconfig import DEFAULT_FILENAME
from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig

DEBUG=False

SKIP_FIRST_GRABS=10 # Skip this many grabs before using one. Needed for D435, it seems.

class LiveGrabber:
    grabber: Optional[cwipc_tiledsource_abstract]

    def __init__(self, captureCreator : cwipc_source_factory_abstract):
        self.cameraconfig = None
        self.grabber = None
        self.captureCreator = captureCreator
        
    def open(self) -> bool:
        self.cameraconfig = CameraConfig(DEFAULT_FILENAME, read=False)
        
        try:
            grabber = self.captureCreator()
            assert hasattr(grabber, 'maxtile')
            self.grabber = cast(cwipc_tiledsource_abstract, grabber)
        except CwipcError as exc:
            print(f'Error opening camera: {exc}', file=sys.stderr)
            return False
        jsonConfig = self.grabber.get_config()
        self.cameraconfig.loadConf(jsonConfig)
        # May need to grab a few combined pointclouds and throw them away
        for i in range(SKIP_FIRST_GRABS):
            pc = self.grabber.get()
            assert pc
            pc.free()
        return True
   
    def __del__(self):
        if self.grabber: self.grabber.free()
        self.grabber = None
    
    def getserials(self):
        """Get serial numbers of cameras"""
        assert self.grabber
        rv = []
        ntile = self.grabber.maxtile()
        for i in range(ntile):
            info = self.grabber.get_tileinfo_dict(i)
            if info.get('cameraName') != None:
                cam_id = info['cameraName']
                cam_id = cam_id.decode('ascii')
                print('Found camera at tile', i, ', camera serial', cam_id)
                rv.append(cam_id)
        return rv
        
    def getmatrix(self, tilenum):
        assert self.cameraconfig
        return self.cameraconfig.getmatrix(tilenum)
        
    def getcount(self) -> int:
        # Get the number of cameras and their tile numbers
        assert self.grabber
        tiles = []
        maxtile = self.grabber.maxtile()
        if DEBUG: print('maxtile', maxtile)
        if maxtile == 1:
            return 1
        else:
            for i in range(1, maxtile):
                info = self.grabber.get_tileinfo_dict(i)
                if DEBUG: print('info', i, info)
                if info != None and info['ncamera'] == 1:
                    tiles.append(i)
        # Check that the tile numbers or a bitmap, as we expect (in join, for example)
        for i in range(len(tiles)):
            assert tiles[i] == (1<<i)
        return len(tiles)
        
    def getpointcloud(self):
        assert self.grabber
        pc = self.grabber.get()
        assert pc
        pc = cast(cwipc_wrapper, pc)
        return Pointcloud.from_cwipc(pc)
