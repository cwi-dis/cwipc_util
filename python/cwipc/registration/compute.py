
from typing import List, Optional, Any, Tuple
import numpy as np
import numpy.typing as npt
import scipy.spatial
import open3d
from matplotlib import pyplot as plt
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from .abstract import *

RegistrationResult = open3d.pipelines.registration.RegistrationResult

RegistrationTransformation = npt.ArrayLike # Should be: NDArray[(4,4), float]

class RegistrationComputer(RegistrationAlgorithm):
    """Compute the registration for a pointcloud.

    """

    def __init__(self):
        self.per_camera_tilenum : List[int] = []
        self.per_camera_pointclouds : List[cwipc_wrapper] = []
        self.correspondence = 1 # Distance in meters between candidate points to be matched, so this is a ridiculously large value

    def set_correspondence(self, correspondence) -> None:
        self.correspondence = correspondence

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
            self._get_nparray_for_pc(cam_pc) for cam_pc in self.per_camera_pointclouds if cam_pc != self.our_pointcloud
        ]
        self.other_points_nparray = np.concatenate(other_nparrays)

    def get_result_transformation(self) -> RegistrationTransformation:
        return np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
    
    def get_result_pointcloud(self) -> cwipc_wrapper:
        pc = self.our_pointcloud
        transform = self.get_result_transformation()
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
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        part_pc = self.get_result_pointcloud()
        for other_pc in self.per_camera_pointclouds:
            if other_pc != self.our_pointcloud:
                new_part_pc = cwipc_join(part_pc, other_pc)
                part_pc.free()
                part_pc = new_part_pc
        return part_pc

class RegistrationComputer_ICP(RegistrationComputer):

    def run(self, target: Optional[int]=None) -> None:
        """Run the algorithm"""
        assert not target is None
        assert len(self.per_camera_pointclouds) > 1
        self._prepare(target)

        initial_transformation = np.identity(4)
        self.registration_result : RegistrationResult = open3d.pipelines.registration.registration_icp(
            source=self._get_source_pointcloud(),
            target=self._get_target_pointcloud(),
            max_correspondence_distance=self._get_max_correspondence_distance(),
            #init=initial_transformation,
            #estimation_method=self._get_estimation_method(),
            criteria=self._get_criteria()
        )

    def get_result_transformation(self) -> RegistrationTransformation:
        print(f"xxxjack {self.registration_result}")
        return self.registration_result.transformation
    
    def _get_source_pointcloud(self) -> open3d.geometry.PointCloud:
        source_pointcloud = open3d.geometry.PointCloud()
        source_pointcloud.points = open3d.utility.Vector3dVector(self.our_points_nparray)
        return source_pointcloud
    
    def _get_target_pointcloud(self) -> open3d.geometry.PointCloud:
        target_pointcloud = open3d.geometry.PointCloud()
        target_pointcloud.points = open3d.utility.Vector3dVector(self.other_points_nparray)
        return target_pointcloud
    
    def _get_max_correspondence_distance(self) -> float:
        return self.correspondence
    
    def _get_estimation_method(self) -> open3d.pipelines.registration.TransformationEstimation:
        estimation_method = open3d.pipelines.registration.TransformationEstimationPointToPoint
        estimation_method.with_scaling = False
        return estimation_method
    
    def _get_criteria(self) -> open3d.pipelines.registration.ICPConvergenceCriteria:
        criteria = open3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness = 1e-7,
            relative_rmse = 1e-7,
            max_iteration = 60
        )
        return criteria