
import copy
import math
from typing import List, Optional, Any, Tuple
import numpy as np
import open3d
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from .abstract import *
from .util import get_tiles_used, o3d_from_cwipc, o3d_pick_points, transformation_identity, cwipc_transform, BaseAlgorithm
from .fine import RegistrationTransformation, RegistrationComputer, RegistrationComputer_ICP_Point2Point

MarkerPosition = Any

class MultiCameraCoarse(MultiAlignmentAlgorithm):
    """Align multiple cameras.
    """

    def __init__(self):
        self.original_pointcloud : Optional[cwipc_wrapper] = None
        self.per_camera_o3d_pointclouds : List[open3d.geometry.PointCloud] = []
        self.per_camera_tilenum : List[int] = []
        self.transformations : List[RegistrationTransformation] = []
        
        self.wanted_marker : MarkerPosition = None
        self.markers : List[MarkerPosition] = []

        self.computer_class = RegistrationComputer_ICP_Point2Point

        self.computer : Optional[RegistrationComputer] = None

        self.verbose = True
  
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
        assert False

    def add_pointcloud(self, pc : cwipc_wrapper) -> int:
        """Add a pointcloud to be used during the algorithm run"""
        assert False
        
    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        assert self.original_pointcloud == None
        self.original_pointcloud = pc

    def camera_count(self) -> int:
        return len(self.per_camera_tilenum)
    
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
        assert self.original_pointcloud
        tilenums = get_tiles_used(self.original_pointcloud)
        for t in tilenums:
            partial_pc = cwipc_tilefilter(self.original_pointcloud, t)
            o3d_partial_pc = o3d_from_cwipc(partial_pc)
            self.per_camera_o3d_pointclouds.append(o3d_partial_pc)
            self.per_camera_tilenum.append(t)
            partial_pc.free()

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        # Initialize the analyzer
        self._prepare()
        # Find the markers in each of the pointclouds
        ok = True
        for o3d_pc in self.per_camera_o3d_pointclouds:
            marker_pos = self._find_marker(o3d_pc)
            if not self._check_marker(marker_pos):
                ok = False
                return False # Or should we continue for the other point clouds? Interactive: no but otherwise?
            self.markers.append(marker_pos)
        
        assert len(self.per_camera_o3d_pointclouds) == len(self.markers)
        indices_to_fix = range(len(self.per_camera_o3d_pointclouds))
        for i in indices_to_fix:
            camnum = self.tilenum_for_camera_index(i)
            o3d_pc = self.per_camera_o3d_pointclouds[i]
            this_marker = self.markers[i]
            this_transform = self._align_marker(camnum, o3d_pc, self.wanted_marker, this_marker)
            if this_transform is None:
                print(f"Error: could not find transform for camera {camnum}")
                ok = False
                this_transform = transformation_identity()
            self.transformations.append(this_transform)
        return ok

    def _check_marker(self, marker : Optional[MarkerPosition]) -> bool:
        return marker != None
    
    def _find_marker(self, pc : open3d.geometry.PointCloud) -> Optional[MarkerPosition]:
        return None
    
    def _align_marker(self, camnum : int, pc : open3d.geometry.PointCloud, target : MarkerPosition, dst : MarkerPosition) -> Optional[RegistrationTransformation]:
        # Create the pointcloud that we want to align to
        target_pc = open3d.geometry.PointCloud()
        target_points = open3d.utility.Vector3dVector(target)
        target_pc.points = target_points
        # Create the pointcloud that we want to align
        dst_pc = open3d.geometry.PointCloud()
        dst_points = open3d.utility.Vector3dVector(dst)
        dst_pc.points = dst_points
        # Create the correspondences
        corr_indices = [(i, i) for i in range(len(dst_points))]
        corr = open3d.utility.Vector2iVector(corr_indices)
        estimator = open3d.pipelines.registration.TransformationEstimationPointToPoint()
        transform = estimator.compute_transformation(dst_pc, target_pc, corr)
        if self.verbose:
            rmse = estimator.compute_rmse(dst_pc, target_pc, corr)
            print(f"cam {camnum}: rmse={rmse}")
        return transform

    def get_result_transformations(self) -> List[RegistrationTransformation]:
        return self.transformations
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        rv : Optional[cwipc_wrapper] = None
        assert len(self.transformations) == len(self.per_camera_tilenum)
        assert self.original_pointcloud
        indices_to_join = range(len(self.per_camera_tilenum))
        
        for i in indices_to_join:
            partial_pc = cwipc_tilefilter(self.original_pointcloud, self.per_camera_tilenum[i])
            transformed_partial_pc = cwipc_transform(partial_pc, self.transformations[i])
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
            if self.verbose:
                print("Error: no marker found")
            return False
        if len(marker) == 4:
            return True
        if self.verbose:
            print(f"Error: marker has {len(marker)} points, expected 4")
        return False
    
    def _find_marker(self, pc : open3d.geometry.PointCloud) -> Optional[MarkerPosition]:
        indices = o3d_pick_points(self.prompt, pc, from000=True)
        points = []
        for i in indices:
            point = pc.points[i]
            points.append(point)
        rv = points
        return rv
    