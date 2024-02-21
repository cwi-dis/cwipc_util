from abc import ABC, abstractmethod
from typing import List, Any, Tuple
from ..abstract import *
from .abstract import *
from .. import cwipc_wrapper, cwipc_from_points, cwipc_tilefilter
import open3d
import open3d.visualization
import numpy as np
from numpy.typing import NDArray

Point_array_xyz = NDArray[Any]
Point_array_rgb = NDArray[Any]


def transformation_identity() -> RegistrationTransformation:
    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

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
    pointarray = np.ctypeslib.as_array(pc.get_points())
    # Extract the relevant fields (X, Y, Z coordinates)
    tilearray = pointarray['tile']
    unique = np.unique(tilearray)
    rv = unique.tolist()
    return rv
   
def _old_cwipc_transform(pc: cwipc_wrapper, transform : RegistrationTransformation) -> cwipc_wrapper:
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

def cwipc_transform(pc: cwipc_wrapper, transform : RegistrationTransformation) -> cwipc_wrapper:
    pc_points = pc.get_points()
    n_points = len(pc_points)
    np_points = np.ctypeslib.as_array(pc_points)
    ones = np.ones(n_points)
    np_points_xyz1 = np.column_stack([np_points['x'], np_points['y'], np_points['z'], ones])
    # Obscure code ahead. I ended up with this expression by trial and error. We first transpose
    # the source array (so it is 4xN shape, then matrix-multiply into the transformation, then transpose again
    # so we end up with Nx4)
    np_points_transformed = (transform @ np_points_xyz1.transpose()).transpose()
    for i in range(n_points):
        pc_points[i].x = np_points_transformed[i][0]
        pc_points[i].y = np_points_transformed[i][1]
        pc_points[i].z = np_points_transformed[i][2]

    new_pc = cwipc_from_points(pc_points, pc.timestamp())
    new_pc._set_cellsize(pc.cellsize())
    return new_pc


class BaseAlgorithm(Algorithm):
    """Base class for most algorithms, both registration and alignment.

    Allows ading point clouds and inspecting them.
    """

    def __init__(self):
        self.per_camera_tilenum : List[int] = []
        self.per_camera_pointclouds : List[cwipc_wrapper] = []
        self.verbose = False

    def add_tiled_pointcloud(self, pc : cwipc_wrapper) -> None:
        """Add each individual per-camera tile of this pointcloud, to be used during the algorithm run"""
        for tilemask in get_tiles_used(pc):
            tiled_pc = self._get_pc_for_cam(pc, tilemask)
            if tiled_pc == None:
                continue
            if tiled_pc.count() == 0:
                continue
            self.per_camera_pointclouds.append(tiled_pc)
            self.per_camera_tilenum.append(tilemask)

    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        return self.per_camera_tilenum[cam_index]

    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        for i in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[i] == tilenum:
                return i
        assert False, f"Tilenum {tilenum} not known"

    def camera_count(self):
        assert len(self.per_camera_tilenum) == len(self.per_camera_pointclouds)
        return len(self.per_camera_tilenum)

    def _get_pc_for_cam(self, pc : cwipc_wrapper, tilemask : int) -> Optional[cwipc_wrapper]:
        rv = cwipc_tilefilter(pc, tilemask)
        if rv.count() != 0:
            return rv
        rv.free()
        return None

    def _get_nparray_for_pc(self, pc : cwipc_wrapper):
        """xxxjack I think this method shoulnd't exist. We should use open3d pointclouds everywhere."""
        # Get the points (as a cwipc-style array) and convert them to a NumPy array-of-structs
        pointarray = np.ctypeslib.as_array(pc.get_points())
        # Extract the relevant fields (X, Y, Z coordinates)
        xyzarray = pointarray[['x', 'y', 'z']]
        # Turn this into an N by 3 2-dimensional array
        nparray = np.column_stack([xyzarray['x'], xyzarray['y'], xyzarray['z']])
        return nparray
