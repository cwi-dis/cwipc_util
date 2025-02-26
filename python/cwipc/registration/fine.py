
from typing import List, Optional, Any, Tuple
import numpy as np
import numpy.typing as npt
import scipy.spatial
import open3d
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from .abstract import *
from .util import transformation_identity, BaseAlgorithm, cwipc_transform, cwipc_point_numpy_matrix_value_type

RegistrationResult = open3d.pipelines.registration.RegistrationResult

class RegistrationComputer(AlignmentAlgorithm, BaseAlgorithm):
    """Compute the registration for a pointcloud.
    This is the base class, which actually does nothing and always returns a fixed unit matrix.
    """
    correspondence : float
    our_pointcloud : Optional[cwipc_wrapper]
    reference_pointcloud : Optional[cwipc_wrapper]
    our_points_nparray : Optional[cwipc_point_numpy_matrix_value_type]
    reference_points_nparray : Optional[cwipc_point_numpy_matrix_value_type]
    registration_result : RegistrationResult

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.correspondence = 1 # Distance in meters between candidate points to be matched, so this is a ridiculously large value
        self.our_pointcloud = None
        self.our_points_nparray = None
        self.reference_points_nparray = None

    def set_correspondence(self, correspondence) -> None:
        self.correspondence = correspondence

    def set_reference_pointcloud(self, pc : cwipc_wrapper) -> None:
        self.reference_pointcloud = pc

    def run(self, target: Optional[int]=None) -> bool:
        """Run the algorithm"""
        assert not target is None
        assert len(self.per_camera_pointclouds) > 1
        self._prepare(target)
        return True

    def _prepare(self, target : int) -> None:
        for targetIndex in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[targetIndex] == target:
                break
        else:
            assert False, f"target camera {target} not found"
        self.our_pointcloud = self.per_camera_pointclouds[targetIndex]
        self.our_points_nparray = self.our_pointcloud.get_numpy_matrix(onlyGeometry=True)
        # If set_reference_pointcloud() was not called, we'll use the combined point clouds of all other cameras as reference
        if self.reference_pointcloud is None:
            other_nparrays = [
                cam_pc.get_numpy_matrix(onlyGeometry=True) for cam_pc in self.per_camera_pointclouds if cam_pc != self.our_pointcloud
            ]
            self.reference_points_nparray = np.concatenate(other_nparrays)
        else:
            self.reference_points_nparray = self.reference_pointcloud.get_numpy_matrix(onlyGeometry=True)

    def get_result_transformation(self, nonverbose=False) -> RegistrationTransformation:
        return transformation_identity()
    
    def get_result_pointcloud(self) -> cwipc_wrapper:
        pc = self.our_pointcloud
        assert pc is not None
        transform = self.get_result_transformation(nonverbose=True)
        new_pc = cwipc_transform(pc, transform)
        return new_pc
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        part_pc = self.get_result_pointcloud()
        if self.reference_pointcloud is not None:
            new_part_pc = cwipc_join(part_pc, self.reference_pointcloud)
            part_pc.free()
            part_pc = new_part_pc
        else:
            for other_pc in self.per_camera_pointclouds:
                if other_pc != self.our_pointcloud:
                    new_part_pc = cwipc_join(part_pc, other_pc)
                    part_pc.free()
                    part_pc = new_part_pc
        return part_pc

class RegistrationComputer_ICP_Point2Point(RegistrationComputer):
    """Compute registration for a pointcloud using the ICP point-to-point algorithm using only geometry."""

    def run(self, target: Optional[int]=None) -> bool:
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
            estimation_method=self._get_estimation_method(),
            criteria=self._get_criteria()
        )
        return True

    def get_result_transformation(self, nonverbose=False) -> RegistrationTransformation:
        if self.verbose and not nonverbose:
            print(f"{self.__class__.__name__}: {self.__class__.__name__} result: {self.registration_result}")
            print(f"\toverlap: {int(self.registration_result.fitness*100)}%")
            print(f"\tinlier RMSE: {self.registration_result.inlier_rmse:.4f}")
            print(f"\ttransformation:\n{self.registration_result.transformation}")
        return self.registration_result.transformation
    
    def _get_source_pointcloud(self) -> open3d.geometry.PointCloud:
        source_pointcloud = open3d.geometry.PointCloud()
        source_pointcloud.points = open3d.utility.Vector3dVector(self.our_points_nparray)
        return source_pointcloud
    
    def _get_target_pointcloud(self) -> open3d.geometry.PointCloud:
        target_pointcloud = open3d.geometry.PointCloud()
        target_pointcloud.points = open3d.utility.Vector3dVector(self.reference_points_nparray)
        return target_pointcloud
    
    def _get_max_correspondence_distance(self) -> float:
        return self.correspondence
    
    def _get_estimation_method(self) -> open3d.pipelines.registration.TransformationEstimation:
        estimation_method = open3d.pipelines.registration.TransformationEstimationPointToPoint(
            with_scaling=False
        )
        return estimation_method
    
    def _get_criteria(self) -> open3d.pipelines.registration.ICPConvergenceCriteria:
        criteria = open3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness = 1e-7,
            relative_rmse = 1e-7,
            max_iteration = 60
        )
        return criteria
    
class RegistrationComputer_ICP_Point2Plane(RegistrationComputer):
    """Compute registration for a pointcloud using the ICP point-to-plane algorithm using only geometry."""

    def run(self, target: Optional[int]=None) -> bool:
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
            estimation_method=self._get_estimation_method(),
            criteria=self._get_criteria()
        )
        return True

    def get_result_transformation(self, nonverbose=False) -> RegistrationTransformation:
        if self.verbose and not nonverbose:
            print(f"{self.__class__.__name__}: {self.__class__.__name__} result: {self.registration_result}")
            print(f"\toverlap: {int(self.registration_result.fitness*100)}%")
            print(f"\tinlier RMSE: {self.registration_result.inlier_rmse:.4f}")
            print(f"\ttransformation:\n{self.registration_result.transformation}")
        return self.registration_result.transformation
    
    def _get_source_pointcloud(self) -> open3d.geometry.PointCloud:
        source_pointcloud = open3d.geometry.PointCloud()
        source_pointcloud.points = open3d.utility.Vector3dVector(self.our_points_nparray)
        source_pointcloud.estimate_normals(self._get_normal_search_strategy())
        return source_pointcloud
    
    def _get_target_pointcloud(self) -> open3d.geometry.PointCloud:
        target_pointcloud = open3d.geometry.PointCloud()
        target_pointcloud.points = open3d.utility.Vector3dVector(self.reference_points_nparray)
        target_pointcloud.estimate_normals(self._get_normal_search_strategy())
        return target_pointcloud
    
    def _get_normal_search_strategy(self):
        # Jack thinks the 0.02 means "2 cm", which means we're looking for
        # a plane of about 4 cm diameter, which seems about reasonable for a human body.
        # The max_nn=30 comes from the default for point2plane, so we'll assume the
        # open3d people know what they're doing.
        return open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30)
    
    def _get_max_correspondence_distance(self) -> float:
        return self.correspondence
    
    def _get_estimation_method(self) -> open3d.pipelines.registration.TransformationEstimation:
        estimation_method = open3d.pipelines.registration.TransformationEstimationPointToPlane()
        return estimation_method
    
    def _get_criteria(self) -> open3d.pipelines.registration.ICPConvergenceCriteria:
        criteria = open3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness = 1e-7,
            relative_rmse = 1e-7,
            max_iteration = 60
        )
        return criteria
    
DEFAULT_ALIGNMENT_ALGORITHM = RegistrationComputer_ICP_Point2Plane