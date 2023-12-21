
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import open3d
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from .abstract import *
from .util import get_tiles_used, o3d_from_cwipc, o3d_pick_points
from .compute import RegistrationTransformation, transformation_identity, RegistrationComputer, RegistrationComputer_ICP_Point2Point

MarkerPosition = Any

class MultiCameraCoarse(RegistrationAlgorithm):
    """Align multiple cameras.
    """

    def __init__(self):
        self.camera_count = 0
      
        self.tiled_pointclouds : List[cwipc_wrapper] = []
        self.o3d_pointclouds : List[open3d.geometry.PointCloud] = []
        self.per_camera_tilenum : List[int] = []
        self.transformations : List[RegistrationTransformation] = []
        
        self.wanted_marker : MarkerPosition = None
        self.markers : List[MarkerPosition] = []

        self.computer_class = RegistrationComputer_ICP_Point2Point

        self.computer : Optional[RegistrationComputer] = None

        self.verbose = False
  
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        assert False

    def add_pointcloud(self, pc : cwipc_wrapper) -> int:
        """Add a pointcloud to be used during the algorithm run"""
        assert False
        
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        self.tiled_pointclouds.append(pc)


    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        return self.per_camera_tilenum[cam_index]

    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        for i in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[i] == tilenum:
                return i
        assert False, f"Tilenum {tilenum} not known"

    
    def _get_pc_for_cam(self, pc : cwipc_wrapper, tilemask : int) -> Optional[cwipc_wrapper]: # xxxjack needed?
        rv = cwipc_tilefilter(pc, tilemask)
        if rv.count() != 0:
            return rv
        rv.free()
        return None
    
    def _prepare(self):
        for pc in self.tiled_pointclouds:
            tilenums = get_tiles_used(pc)
            for t in tilenums:
                partial_pc = cwipc_tilefilter(pc, t)
                o3d_partial_pc = o3d_from_cwipc(partial_pc)
                self.o3d_pointclouds.append(o3d_partial_pc)
                self.per_camera_tilenum.append(t)
                partial_pc.free()

    def run(self) -> None:
        """Run the algorithm"""
        assert len(self.tiled_pointclouds) == 1
        # Initialize the analyzer
        self._prepare()
        # Find the markers in each of the pointclouds
        ok = True
        for o3d_pc in self.o3d_pointclouds:
            marker_pos = self._find_marker(o3d_pc)
            if not self._check_marker(marker_pos):
                ok = False
            self.markers.append(marker_pos)
        if not ok:
            return
        assert len(self.o3d_pointclouds) == len(self.markers)
        indices_to_fix = range(len(self.o3d_pointclouds))
        for i in indices_to_fix:
            o3d_pc = self.o3d_pointclouds[i]
            this_marker = self.markers[i]
            this_transform = self._align_marker(o3d_pc, self.wanted_marker, this_marker)
            if this_transform == None:
                ok = False
                this_transform = transformation_identity()
            self.transformations.append(this_transform)

    def _check_marker(self, marker : Optional[MarkerPosition]) -> bool:
        return marker != None
    
    def _find_marker(self, pc : open3d.geometry.PointCloud) -> Optional[MarkerPosition]:
        return None
    
    def _align_marker(self, pc : open3d.geometry.PointCloud, dst : MarkerPosition, src : MarkerPosition) -> Optional[RegistrationTransformation]:
        return None

    def get_result_transformations(self) -> List[RegistrationTransformation]:
        return self.transformations
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        rv : Optional[cwipc_wrapper] = None
        assert len(self.transformations) == len(self.per_camera_tilenum)
        assert len(self.tiled_pointclouds) == 1
        original_pc = self.tiled_pointclouds[0]
        indices_to_join = range(len(self.per_camera_tilenum))
        indices_to_join = [0]
       
        for i in indices_to_join:
            partial_pc = cwipc_tilefilter(original_pc, self.per_camera_tilenum[i])
            transformed_partial_pc = self._transform_partial_pc(partial_pc, self.transformations[i])
            partial_pc.free()
            partial_pc = None
            if rv == None:
                rv = transformed_partial_pc
            else:
                new_rv = cwipc_join(rv, transformed_partial_pc)
                rv.free()
                transformed_partial_pc.free()
                rv = new_rv
        assert rv
        return rv

    def _transform_partial_pc(self, pc: cwipc_wrapper, transform : RegistrationTransformation) -> cwipc_wrapper:
        points = pc.get_points()
        for i in range(len(points)):
            point = np.array([
                points[i].x,
                points[i].y,
                points[i].z,
                1
            ])
            transformed_point = transform.dot(point) # type: ignore
            points[i].x = transformed_point[0]
            points[i].y = transformed_point[1]
            points[i].z = transformed_point[2]
        new_pc = cwipc_from_points(points, pc.timestamp())
        new_pc._set_cellsize(pc.cellsize())
        return new_pc
    
class MultiCameraCoarseInteractive(MultiCameraCoarse):

    def __init__(self):
        MultiCameraCoarse.__init__(self)
        self.wanted_marker = [
            [-0.105, 0, +0.148],   # topleft, blue
            [+0.105, 0, +0.148],   # topright, red
            [+0.105, 0, -0.148],  # botright, yellow
            [-0.105, 0, -0.148],  # botleft, pink
        ]
        self.prompt = "Select blue, red, yellow and pink corners (in that order)"

    def _check_marker(self, marker : Optional[MarkerPosition]) -> bool:
        if marker == None:
            return False
        return len(marker) == 4
    
    def _find_marker(self, pc : open3d.geometry.PointCloud) -> Optional[MarkerPosition]:
        indices = o3d_pick_points(self.prompt, pc, from000=True)
        points = []
        for i in indices:
            point = pc.points[i]
            points.append(point)
        rv = points
        return rv
    