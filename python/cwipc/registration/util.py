from abc import ABC, abstractmethod
from typing import List, Any, Tuple
try:
    from typing import override
except ImportError:
    from typing_extensions import override
from ..abstract import *
from .abstract import *
from .. import cwipc_wrapper, cwipc_from_points, cwipc_from_numpy_array, cwipc_from_numpy_matrix, cwipc_tilefilter
import open3d
import open3d.visualization
import numpy as np
from numpy.typing import NDArray
import textwrap

def algdoc(klass : type, indent : int) -> str:
    doc = klass.__doc__
    if doc == None:
        doc = "No documentation available"
    doc = textwrap.dedent(doc)
    doc = textwrap.indent(doc, '\t'*indent)
    return doc

Point_array_xyz = NDArray[Any]
Point_array_rgb = NDArray[Any]


def transformation_identity() -> RegistrationTransformation:
    values  = np.array([
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ], dtype=float)
    return np.reshape(values, (4, 4))

def transformation_invert(orig_transform : RegistrationTransformation) -> RegistrationTransformation:
    """Invert an affine transformation"""
    # Get the 3x3 linear transformation and the translation vector
    orig_matrix = orig_transform[:3, :3]
    orig_translation = orig_transform[:3, 3]
    # Get the inverse linear transformation and the inverse translation
    # Note that we can invert the matrix by transposing because we know it is size-preserving.
    # Also note that the translation vector is in "new coordinates"
    inv_matrix = orig_matrix.T
    inv_translation = -inv_matrix @ orig_translation
    # Put everything together into a 4x4 matrix
    transform = np.empty((4, 4))
    transform[:3, :3] = inv_matrix
    transform[:3, 3] = inv_translation
    transform[3, :] = [0, 0, 0, 1]
    return transform

def transformation_frompython(trafo : List[List[float]]) -> RegistrationTransformation:
    rv = np.array(trafo)
    assert rv.shape == (4, 4)
    return rv

def transformation_topython(matrix : RegistrationTransformation) -> List[List[float]]:
    rv = matrix.tolist()
    assert len(rv) == 4
    assert len(rv[0]) == 4
    return rv

def cwipc_center(pc : cwipc_wrapper) -> Tuple[float, float, float]:
    """Compute the center of a point cloud"""
    point_matrix = pc.get_numpy_matrix()
    points = point_matrix[:, :3]
    centroid = np.mean(points, axis=0)
    return tuple(centroid)

def cwipc_tilefilter_masked(pc : cwipc_wrapper, mask : int) -> cwipc_wrapper:
    """Filter a point cloud for specific tiles. Each point tile number is ANDed to the mask, so only points with a tile number that matches the mask are returned.
    The mask is a bitmask."""
    pointarray = pc.get_numpy_array()
    # Extract the relevant fields (X, Y, Z coordinates)
    tilearray = pointarray['tile']
    # Create a mask for the points that match the tile mask
    mask_array = (tilearray & mask) != 0
    # Filter the point cloud using the mask
    filtered_points = pointarray[mask_array]
    if filtered_points.size == 0:
        return cwipc_from_points([], pc.timestamp())
    new_pc = cwipc_from_numpy_array(filtered_points, pc.timestamp())
    new_pc._set_cellsize(pc.cellsize())
    return new_pc

def show_pointcloud(title : str, pc : Union[cwipc_wrapper, open3d.geometry.PointCloud], from000=False):
    """Show a point cloud in a window. Allow user to interact with it until q is pressed, at which point this method returns.
    
    The point cloud can be either a cwipc or an opend3.geometry.PointCloud.

    The optional from000 argument places the camera at (0, 0, 0).
    """
    if type(pc) == cwipc_wrapper:
        pc_o3d = pc.get_o3d_pointcloud()
        o3d_show_points(title, pc_o3d, from000)
    else:
        o3d_show_points(title, pc, from000)

def o3d_pick_points(title : str, pc : open3d.geometry.PointCloud, from000 : bool=False) -> List[int]:
    """Show a window with an open3d.geometry.PointCloud. Let the user pick points and return the list of point indices picked."""
    vis = open3d.visualization.VisualizerWithEditing() # type: ignore
    vis.create_window(window_name=title, width=1280, height=720)
    #self.winpos += 50
    vis.add_geometry(pc)
    if from000:
        viewControl = vis.get_view_control()
        pinholeCamera = viewControl.convert_to_pinhole_camera_parameters()
        pinholeCamera.extrinsic = transformation_identity()
        viewControl.convert_from_pinhole_camera_parameters(pinholeCamera)
    vis.run() # user picks points
    vis.destroy_window()
    return vis.get_picked_points()

def o3d_show_points(title : str, pc : open3d.geometry.PointCloud, from000=False, keepopen=False) -> Optional[open3d.visualization.Visualizer]:
    """Show a window with an open3d.geometry.PointCloud. """
    vis = open3d.visualization.Visualizer() # type: ignore
    vis.create_window(window_name=title) 
    vis.add_geometry(pc)
    DRAW_OWN_AXES = False
    if DRAW_OWN_AXES:
        # Draw 1 meter axes (x=red, y=green, z=blue)
        axes = open3d.geometry.LineSet()
        axes.points = open3d.utility.Vector3dVector([[0,0,0], [1,0,0], [0,1,0], [0,0,1]])
        axes.lines = open3d.utility.Vector2iVector([[0,1], [0,2], [0,3]])
        axes.colors = open3d.utility.Vector3dVector([[1,0,0], [0,1,0], [0,0,1]])
    else:
        axes = open3d.geometry.TriangleMesh.create_coordinate_frame()
    vis.add_geometry(axes)
    if from000:
        viewControl = vis.get_view_control()
        pinholeCamera = viewControl.convert_to_pinhole_camera_parameters()
        pinholeCamera.extrinsic = transformation_identity()
        viewControl.convert_from_pinhole_camera_parameters(pinholeCamera)
    vis.run()
    if keepopen:
        return vis
    vis.destroy_window()
    return None

def get_tiles_used(pc : cwipc_wrapper) -> List[int]:
    """Return a list of the tile numbers used in the point cloud"""
    pointarray = pc.get_numpy_array()
    # Extract the relevant fields (X, Y, Z coordinates)
    tilearray = pointarray['tile']
    unique = np.unique(tilearray)
    rv : List[int] = unique.tolist()
    rv.sort()
    return rv

def cwipc_transform(pc: cwipc_wrapper, transform : RegistrationTransformation) -> cwipc_wrapper:
    """Apply an affine transpormation to a point cloud and return the resulting point cloud"""

    np_points = pc.get_numpy_matrix()
    n_points = np_points.shape[0]
    np_points_xyz = np_points[...,0:3]
    # Split the affine transform into a rotation and a translation
    rotmat = transform[:3,:3]
    transvec = transform[:3,3].transpose()
    np_points_transformed = (rotmat @ np_points_xyz.transpose()).transpose()
    np_points_transformed = np_points_transformed + transvec
    np_points[...,0:3] = np_points_transformed
    new_pc = cwipc_from_numpy_matrix(np_points, pc.timestamp())
    new_pc._set_cellsize(pc.cellsize())
    return new_pc

class BaseAlgorithm(Algorithm):
    """Base class for most algorithms, both registration and alignment.

    Allows ading point clouds and inspecting them.
    """

    def __init__(self):
        self.source_pointcloud : Optional[cwipc_wrapper] = None
        self.source_tilemask : Optional[int] = None
        self.reference_pointcloud : Optional[cwipc_wrapper] = None
        self.reference_tilemask : Optional[int] = None
        self.verbose = False

    @override
    def set_source_pointcloud(self, pc : cwipc_wrapper, tilemask: Optional[int] = None) -> None:
        """Set the source point cloud for this algorithm"""
        if tilemask is not None:
            pre_count = pc.count()
            if tilemask != 0:
                pc = cwipc_tilefilter_masked(pc, tilemask)
            post_count = pc.count()
            if self.verbose:
                print(f"{self.__class__.__name__}: Setting source point cloud with {post_count} (of {pre_count}) points using tilemask {tilemask:#x}")
        else:
            if self.verbose:
                print(f"{self.__class__.__name__}: Setting source point cloud with {pc.count()} points")

        self.source_pointcloud = pc
        self.source_tilemask = tilemask
    
    @override
    def set_reference_pointcloud(self, pc : cwipc_wrapper, tilemask : Optional[int] = None) -> None:
        """Set the reference point cloud for this algorithm"""
        if tilemask is not None:
            pre_count = pc.count()
            if tilemask != 0:
                pc = cwipc_tilefilter_masked(pc, tilemask)
            post_count = pc.count()
            if self.verbose:
                print(f"{self.__class__.__name__}: Setting target point cloud with {post_count} (of {pre_count}) points using tilemask {tilemask:#x}")
        else:
            if self.verbose:
                print(f"{self.__class__.__name__}: Setting target point cloud with {pc.count()} points")
        self.reference_pointcloud = pc
        self.reference_tilemask = tilemask

    
class BaseMulticamAlgorithm(MulticamAlgorithm):
    """Base class for most algorithms, both registration and alignment.

    Allows ading point clouds and inspecting them.
    """

    def __init__(self):
        self.per_camera_tilenum : List[int] = []
        self.original_pointcloud : Optional[cwipc_wrapper] = None
        self.verbose = False

    @override
    def set_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        self.original_pointcloud = pc
        for tilemask in get_tiles_used(pc):
            self.per_camera_tilenum.append(tilemask)

    @override
    def tilemask_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        return self.per_camera_tilenum[cam_index]

    @override
    def camera_index_for_tilemask(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        for i in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[i] == tilenum:
                return i
        assert False, f"Tilenum {tilenum} not known"

    @override
    def camera_count(self):
        return len(self.per_camera_tilenum)

#    def _get_pc_for_cam(self, pc : cwipc_wrapper, tilemask : int) -> Optional[cwipc_wrapper]:
#        rv = cwipc_tilefilter(pc, tilemask)
#        if rv.count() != 0:
#            return rv
#        rv.free()
#        return None

#    @override
#    def get_pointcloud_for_tilenum(self, tilenum : int) -> cwipc_wrapper:
#        """Returns the point cloud for this tilenumber"""
#        cam_index = self.camera_index_for_tilenum(tilenum)
#        return self.per_camera_pointclouds[cam_index]
