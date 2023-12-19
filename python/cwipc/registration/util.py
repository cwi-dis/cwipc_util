from abc import ABC, abstractmethod
from typing import List, Any, Tuple
from ..abstract import *
from .abstract import *
from .. import cwipc_wrapper
import open3d
import open3d.visualization
import numpy as np
from numpy.typing import NDArray

Point_array_xyz = NDArray[Any]
Point_array_rgb = NDArray[Any]

def show_pointcloud(title : str, pc : Union[cwipc_wrapper, open3d.geometry.PointCloud], from000=False):
    """Show a point cloud in a window. Allow user to interact with it until q is pressed, at which point this method returns.
    
    The point cloud can be either a cwipc or an opend3.geometry.PointCloud.

    The optional from000 argument places the camera at (0, 0, 0).
    """
    if type(pc) == cwipc_wrapper:
        pc_o3d = o3d_from_cwipc(pc)
        o3d_show_points(title, pc_o3d, from000)
    else:
        o3d_show_points(title, pc, from000)

def o3d_from_cwipc(pc : cwipc_wrapper) -> open3d.geometry.PointCloud:
    """Convert a cwipc point cloud to a open3d.geometry.PointCloud"""
    np_xyz_array, np_rgb_array = nparrays_from_cwipc(pc)
    points_v = open3d.utility.Vector3dVector(np_xyz_array)
    colors_v = open3d.utility.Vector3dVector(np_rgb_array)
    o3d_pc = open3d.geometry.PointCloud()
    o3d_pc.points = points_v
    o3d_pc.colors = colors_v
    return o3d_pc

def nparray_xyz_from_cwipc(pc : cwipc_wrapper) -> Point_array_xyz:
    """Return the [X, Y, Z] numpy array for a cwipc point cloud"""
    # Get the points (as a cwipc-style array) and convert them to a NumPy array-of-structs
    pointarray = np.ctypeslib.as_array(pc.get_points())
    # Extract the relevant fields (X, Y, Z coordinates)
    xyzarray = pointarray[['x', 'y', 'z']]
    # Turn this into an N by 3 2-dimensional array
    nparray = np.column_stack([xyzarray['x'], xyzarray['y'], xyzarray['z']])
    return nparray

def nparrays_from_cwipc(pc : cwipc_wrapper) -> Tuple[Point_array_xyz, Point_array_rgb]:
    """For the cwipc argument, return two numpy arrays: one with the XYZ coordinates, one with the RGB colors (as normalized float32)"""
    # Get the points (as a cwipc-style array) and convert them to a NumPy array-of-structs
    pointarray = np.ctypeslib.as_array(pc.get_points())
    # Extract the relevant fields (X, Y, Z coordinates)
    xyzarray = pointarray[['x', 'y', 'z']]
    
    # Turn this into an N by 3 2-dimensional array
    np_xyz_array = np.column_stack([pointarray['x'], pointarray['y'], pointarray['z']])
    np_rgb_array_u8 = np.column_stack([pointarray['r'], pointarray['g'], pointarray['b']])
    np_rgb_array = np_rgb_array_u8.astype(np.float32) / 255.0
    return np_xyz_array, np_rgb_array

def o3d_pick_points(title : str, pc : open3d.geometry.PointCloud, from000 : bool=False) -> List[int]:
    """Show a window with an open3d.geometry.PointCloud. Let the user pick points and return the list of point indices picked."""
    vis = open3d.visualization.VisualizerWithEditing() # type: ignore
    vis.create_window(window_name=title, width=960, height=540)
    #self.winpos += 50
    vis.add_geometry(pc)
    if from000:
        viewControl = vis.get_view_control()
        viewControl.set_up([0, -1, 0])
        viewControl.set_front([0, 0, -1])
        viewControl.set_lookat([0, 0, 0])
    vis.run() # user picks points
    vis.destroy_window()
    return vis.get_picked_points()

def o3d_show_points(title : str, pc : open3d.geometry.PointCloud, from000=False) -> None:
    """Show a window with an open3d.geometry.PointCloud. """
    vis = open3d.visualization.Visualizer() # type: ignore
    vis.create_window(window_name=title, width=960, height=540) # xxxjack: , left=self.winpos, top=self.winpos
    #self.winpos += 50
    vis.add_geometry(pc)
    DRAW_OWN_AXES = True
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
        viewControl.set_up([0, -1, 0])
        viewControl.set_front([0, 0, -1])
        viewControl.set_lookat([0, 0, 0])
    vis.run()
    vis.destroy_window()
    
def get_tiles_used(pc : cwipc_wrapper) -> List[int]:
    """Return a list of the tile numbers used in the point cloud"""
    pointarray = np.ctypeslib.as_array(pc.get_points())
    # Extract the relevant fields (X, Y, Z coordinates)
    tilearray = pointarray['tile']
    unique = np.unique(tilearray)
    rv = unique.tolist()
    return rv
   