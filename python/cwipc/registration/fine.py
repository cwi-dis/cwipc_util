
from typing import List, Optional, Any, Tuple
try:
    from typing import override
except ImportError:
    from typing_extensions import override
import numpy as np
import numpy.typing as npt
import scipy.spatial
import open3d
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from ..util import cwipc_point_numpy_matrix_value_type
from .abstract import *
from .util import transformation_identity, BaseAlgorithm, cwipc_transform, algdoc

RegistrationResult = Any # open3d.pipelines.registration.RegistrationResult

class RegistrationComputer(AlignmentAlgorithm, BaseAlgorithm):
    """
    Compute the registration for a pointcloud.
    This is the base class, which actually does nothing and always returns a fixed unit matrix.
    """
    correspondence : float
    our_points_nparray : Optional[cwipc_point_numpy_matrix_value_type]
    reference_points_nparray : Optional[cwipc_point_numpy_matrix_value_type]
    registration_result : RegistrationResult

    def __init__(self):
        BaseAlgorithm.__init__(self)
        self.correspondence = 0 # Distance in meters between candidate points to be matched, so this is a ridiculously large value
        self.our_points_nparray = None
        self.reference_points_nparray = None

    @override
    def set_correspondence(self, correspondence) -> None:
        self.correspondence = correspondence

    @override
    def run(self) -> bool:
        """Run the algorithm"""
        self._prepare()
        return True

    def _prepare(self) -> None:
        assert self.source_pointcloud is not None, "source point cloud must be set before running the algorithm"
        assert self.reference_pointcloud is not None, "reference point cloud must be set before running the algorithm"
        self.our_points_nparray = self.source_pointcloud.get_numpy_matrix(onlyGeometry=True)
        # If set_reference_pointcloud() was not called, we'll use the combined point clouds of all other cameras as reference
        self.reference_points_nparray = self.reference_pointcloud.get_numpy_matrix(onlyGeometry=True)
        if self.verbose:
            print(f"{self.__class__.__name__}: with {len(self.our_points_nparray)} points and {len(self.reference_points_nparray)} reference points")
        self._compute_centroids()

    def _compute_centroids(self) -> None:
        assert not self.our_points_nparray is None
        assert not self.reference_points_nparray is None
        our_centroid = np.mean(self.our_points_nparray, axis=0)
        reference_centroid = np.mean(self.reference_points_nparray, axis=0)
        self.our_centroid = our_centroid
        self.reference_centroid = reference_centroid
        our_centroid[1] = 0
        reference_centroid[1] = 0
        if self.correspondence == 0:
            # If correspondence is not set use distance between the (floor-based) centroids.
            self.correspondence = float(np.linalg.norm(our_centroid - reference_centroid)) / 2
            if self.verbose:
                print(f"{self.__class__.__name__}: set correspondence to {self.correspondence:.4f} meters")
    
    @override
    def get_result_transformation(self, nonverbose=False) -> RegistrationTransformation:
        return transformation_identity()
    
    @override
    def get_result_pointcloud(self) -> cwipc_wrapper:
        pc = self.source_pointcloud
        assert pc is not None
        transform = self.get_result_transformation(nonverbose=True)
        new_pc = cwipc_transform(pc, transform)
        return new_pc
    
    @override
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        assert self.reference_pointcloud is not None
        part_pc = self.get_result_pointcloud()
        new_part_pc = cwipc_join(part_pc, self.reference_pointcloud)
        part_pc.free()
        part_pc = new_part_pc
        return part_pc

class RegistrationComputer_ICP_Point2Point(RegistrationComputer):
    """
    Compute registration for a pointcloud using the ICP point-to-point algorithm using only geometry.
    """

    @override
    def run(self) -> bool:
        """Run the algorithm"""
        self._prepare()

        initial_transformation = np.identity(4)
        self.registration_result : RegistrationResult = open3d.pipelines.registration.registration_icp(
            source=self._get_source_pointcloud(),
            target=self._get_target_pointcloud(),
            max_correspondence_distance=self.correspondence,
            #init=initial_transformation,
            estimation_method=self._get_estimation_method(),
            criteria=self._get_criteria()
        )
        return True

    @override
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
    
    def _get_estimation_method(self) -> open3d.pipelines.registration.TransformationEstimation:
        estimation_method = open3d.pipelines.registration.TransformationEstimationPointToPoint(
            with_scaling=False
        )
        return estimation_method
    
    def _get_criteria(self) -> open3d.pipelines.registration.ICPConvergenceCriteria:
        criteria = open3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness = 1e-3,
            relative_rmse = 1e-6,
            max_iteration = 30
        )
        return criteria
    
class RegistrationComputer_Tensor_ICP_Point2Point(RegistrationComputer):
    """
    Compute registration for a pointcloud using the ICP point-to-point algorithm using only geometry.
    """

    @override
    def run(self) -> bool:
        """Run the algorithm"""
        self._prepare()

        initial_transformation = np.identity(4)
        self.registration_result : RegistrationResult = open3d.t.pipelines.registration.icp(
            source=self._get_source_tensor_pointcloud(),
            target=self._get_target_tensor_pointcloud(),
            max_correspondence_distance=self.correspondence,
            #init=initial_transformation,
            estimation_method=self._get_estimation_method(),
            criteria=self._get_criteria(),
            callback_after_iteration=self._callback_after_iteration
        )
        return True
    
    def _callback_after_iteration(self, loss_log_map : Any) -> None:
        if not self.verbose:
            return
        index = loss_log_map['iteration_index'].item()
        fitness = loss_log_map['fitness'].item()
        inlier_rmse = loss_log_map['inlier_rmse'].item()
        print(f"iteration {index}: fitness={fitness:.4f}, inlier_rmse={inlier_rmse:.4f}")

    def _get_source_tensor_pointcloud(self) -> open3d.t.geometry.PointCloud:
        np_points = self.our_points_nparray
        tensor_points = open3d.core.Tensor(np_points)
        tensor_pointcloud = open3d.t.geometry.PointCloud(tensor_points)
        return tensor_pointcloud
    
    def _get_target_tensor_pointcloud(self) -> open3d.t.geometry.PointCloud:
        np_points = self.reference_points_nparray
        tensor_points = open3d.core.Tensor(np_points)
        tensor_pointcloud = open3d.t.geometry.PointCloud(tensor_points)
        return tensor_pointcloud
    
    @override
    def get_result_transformation(self, nonverbose=False) -> RegistrationTransformation:
        if self.verbose and not nonverbose:
            print(f"{self.__class__.__name__}: {self.__class__.__name__} result: {self.registration_result}")
            print(f"\toverlap: {int(self.registration_result.fitness*100)}%")
            print(f"\tinlier RMSE: {self.registration_result.inlier_rmse:.4f}")
            print(f"\ttransformation:\n{self.registration_result.transformation}")
        tensor_transformation = self.registration_result.transformation
        return tensor_transformation.numpy()
    
    def _get_source_pointcloud(self) -> open3d.geometry.PointCloud:
        assert False
        source_pointcloud = open3d.geometry.PointCloud()
        source_pointcloud.points = open3d.utility.Vector3dVector(self.our_points_nparray)
        return source_pointcloud
    
    def _get_target_pointcloud(self) -> open3d.geometry.PointCloud:
        assert False
        target_pointcloud = open3d.geometry.PointCloud()
        target_pointcloud.points = open3d.utility.Vector3dVector(self.reference_points_nparray)
        return target_pointcloud
    
    def _get_estimation_method(self) -> open3d.pipelines.registration.TransformationEstimation:
        estimation_method = open3d.t.pipelines.registration.TransformationEstimationPointToPoint(
        )
        return estimation_method
    
    def _get_criteria(self) -> open3d.pipelines.registration.ICPConvergenceCriteria:
        criteria = open3d.t.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness = 1e-7,
            relative_rmse = 1e-7,
            max_iteration = 60
        )
        return criteria
    
class RegistrationComputer_ICP_Point2Plane(RegistrationComputer):
    """
    Compute registration for a pointcloud using the ICP point-to-plane algorithm using only geometry.
    """

    @override
    def run(self) -> bool:
        """Run the algorithm"""
        self._prepare()

        initial_transformation = np.identity(4)
        self.registration_result = open3d.pipelines.registration.registration_icp(
            source=self._get_source_pointcloud(),
            target=self._get_target_pointcloud(),
            max_correspondence_distance=self.correspondence,
            #init=initial_transformation,
            estimation_method=self._get_estimation_method(),
            criteria=self._get_criteria()
        )
        return True

    @override
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
    
DEFAULT_FINE_ALIGNMENT_ALGORITHM = RegistrationComputer_ICP_Point2Plane

ALL_FINE_ALIGNMENT_ALGORITHMS = [
    RegistrationComputer_ICP_Point2Point,
    RegistrationComputer_ICP_Point2Plane
]

HELP_FINE_ALIGNMENT_ALGORITHMS = """
## Fine alignment algorithms

The alignment algorithm looks at a source point cloud and tries to find the best transformation to align it with a target point cloud.

The default fine alignment algorithm is """ + DEFAULT_FINE_ALIGNMENT_ALGORITHM.__name__ + """

The following alignment algorithms are available:
""" + "\n".join([f"\t{alg.__name__}\n{algdoc(alg, 2)}" for alg in ALL_FINE_ALIGNMENT_ALGORITHMS])