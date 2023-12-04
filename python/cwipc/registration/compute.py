
from typing import List, Optional, Any, Tuple
import numpy as np
import scipy.spatial
from matplotlib import pyplot as plt
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from .abstract import *

RegistrationTransformation = Tuple[float, float, float] # xxxjack for now: translation only

class RegistrationComputer(RegistrationAlgorithm):
    """Compute the registration for a pointcloud.

    """

    def __init__(self):
        self.per_camera_tilenum : List[int] = []
        self.per_camera_pointclouds : List[cwipc_wrapper] = []

    def add_pointcloud(self, pc : cwipc_wrapper) -> int:
        """Add a pointcloud to be used during the algorithm run"""
        tilenum = 1000+len(self.per_camera_pointclouds)
        self.per_camera_tilenum.append(tilenum)
        self.per_camera_pointclouds.append(pc)
        return tilenum
        
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        for tilemask in [1,2,4,8,16,32,64,128]:
            tiled_pc = self._get_pc_for_cam(pc, tilemask)
            if tiled_pc == None:
                continue
            if tiled_pc.count() == 0:
                continue
            self.per_camera_pointclouds.append(tiled_pc)
            self.per_camera_tilenum.append(tilemask)
   
    def _get_pc_for_cam(self, pc : cwipc_wrapper, tilemask : int) -> Optional[cwipc_wrapper]:
        rv = cwipc_tilefilter(pc, tilemask)
        if rv.count() != 0:
            return rv
        rv.free()
        return None

    def _get_nparray_for_pc(self, pc : cwipc_wrapper):
        # Get the points (as a cwipc-style array) and convert them to a NumPy array-of-structs
        pointarray = np.ctypeslib.as_array(pc.get_points())
        # Extract the relevant fields (X, Y, Z coordinates)
        xyzarray = pointarray[['x', 'y', 'z']]
        # Turn this into an N by 3 2-dimensional array
        nparray = np.column_stack([xyzarray['x'], xyzarray['y'], xyzarray['z']])
        return nparray

    def run(self, target: Optional[int]=None) -> None:
        """Run the algorithm"""
        assert not target is None
        assert len(self.per_camera_pointclouds) > 1
        self._prepare(target)
        pass

    def _prepare(self, target : int) -> None:
        for targetIndex in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[targetIndex] == target:
                break
        else:
            assert False, f"target camera {target} not found"
        self.our_pointcloud = self.per_camera_pointclouds[targetIndex]
        self.our_points_nparray = self._get_nparray_for_pc(self.our_pointcloud)

        other_nparrays = [
            self._get_nparray_for_pc(cam_pc) for cam_pc in self.per_camera_pointclouds if cam_pc != targetIndex
        ]
        self.other_points_nparray = np.concatenate(other_nparrays)

    def get_result_transformation(self) -> RegistrationTransformation:
        return (0.0, 0.0, 0.0)
    
    def get_result_pointcloud(self) -> cwipc_wrapper:
        pc = self.our_pointcloud
        transform = self.get_result_transformation()
        points = pc.get_points()
        for i in range(len(points)):
            points[i].x += transform[0]
            points[i].y += transform[1]
            points[i].z += transform[2]
        new_pc = cwipc_from_points(points, pc.timestamp())
        new_pc._set_cellsize(pc.cellsize())
        return new_pc
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        part_pc = self.get_result_pointcloud()
        for other_pc in self.per_camera_pointclouds:
            if other_pc != self.our_pointcloud:
                new_part_pc = cwipc_join(part_pc, other_pc)
                part_pc.free()
                part_pc = new_part_pc
        return part_pc
