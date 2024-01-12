
import copy
import math
from typing import List, Optional, Any, Tuple, Sequence, cast, Dict
import numpy as np
from numpy.typing import NDArray
import open3d
import open3d.visualization
import cv2.typing
import cv2.aruco
from .. import cwipc_wrapper, cwipc_tilefilter, cwipc_from_points, cwipc_join
from .abstract import *
from .util import get_tiles_used, o3d_from_cwipc, o3d_pick_points, o3d_show_points, transformation_identity, transformation_invert, cwipc_transform, BaseAlgorithm
from .fine import RegistrationTransformation, RegistrationComputer, RegistrationComputer_ICP_Point2Point

MarkerPosition = List[Tuple[float, float, float]] # Position (outline) of a marker in 3D coordinates
MarkerPositions = Dict[int, MarkerPosition]   # map marks IDs to positions

class MultiCameraCoarse(MultiAlignmentAlgorithm):
    """Align multiple cameras.
    """

    def __init__(self):
        self.debug = False
        self.original_pointcloud : Optional[cwipc_wrapper] = None
        self.per_camera_o3d_pointclouds : List[open3d.geometry.PointCloud] = []
        self.per_camera_tilenum : List[int] = []
        self.transformations : List[RegistrationTransformation] = []
        
        self.known_marker_positions : MarkerPositions = dict()
        self.markers : List[MarkerPositions] = []

        self.computer_class = RegistrationComputer_ICP_Point2Point

        self.computer : Optional[RegistrationComputer] = None

        self.verbose = True
  
    def plot(self, filename : Optional[str]=None, show : bool = False, cumulative : bool = False):
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

    
    def _prepare(self):
        """From the point cloud added prepare the data structures to run the algorithm"""
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
        # xxxjack this should be done differently. This is an all-or-nothing strategy, where we find the
        # markers in all captures or we fail.
        #
        # In stead we should be happy with partial matches, for some captures, and align those. We can then
        # do another grab and maybe fix the missing ones.
        #
        # Also, this would help a lot with detecting non-overlap[ping captures with multiple markers.
        for o3d_pc in self.per_camera_o3d_pointclouds:
            markers = self._find_markers(o3d_pc)
            self.markers.append(markers)
        
        assert len(self.per_camera_o3d_pointclouds) == len(self.markers)
        indices_to_fix = range(len(self.per_camera_o3d_pointclouds))
        for i in indices_to_fix:
            camnum = self.tilenum_for_camera_index(i)
            o3d_pc = self.per_camera_o3d_pointclouds[i]
            this_markers = self.markers[i]
            assert self.known_marker_positions
            found_new_markers = False
            found_existing_markers = False
            for id, area in this_markers.items():
                if not self._check_marker(area):
                    continue
                if id in self.known_marker_positions:
                    # This is a marker we know. Align it.
                    wanted_marker_corners = self.known_marker_positions[id]
                    this_marker_corners = area
                    this_transform = self._align_marker(camnum, o3d_pc, wanted_marker_corners, this_marker_corners)
                    if not this_transform is None:
                        if found_existing_markers:
                            # We already found one....
                            print("Warning: found another marker that we already know")
                        else:
                            found_existing_markers = True
                            self.transformations.append(this_transform)
                else:
                    print(f"Found a new marker with id={id}")
                    found_new_markers = True
            if found_new_markers and found_existing_markers:
                print(f"xxxjack Found new markers. Should add them to our collection")
            if not found_existing_markers:
                # Nothing found. Add idnetity transformation
                self.transformations.append(transformation_identity())
        return ok

    def _check_marker(self, marker : MarkerPosition) -> bool:
        """Return False if the MarkerPosition cannot be a valid marker"""
        if len(marker) == 4:
            return True
        print(f"Error: marker has {len(marker)} corners in stead of 4")
        return False
    
    def _find_markers(self, pc : open3d.geometry.PointCloud) -> MarkerPositions:
        """Return a dictionary of all markers found in the point cloud (indexed by marker ID)"""
        return {}
    
    def _align_marker(self, camnum : int, pc : open3d.geometry.PointCloud, target : MarkerPosition, dst : MarkerPosition) -> Optional[RegistrationTransformation]:
        """Find the transformation that will align pc so that the target marker matches best with the dst marker"""
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
            if self.verbose:
                print(f"_align_marker: cam {camnum}: rmse={rmse}")
        return transform

    def get_result_transformations(self) -> List[RegistrationTransformation]:
        """Return the transformations found, indexed by camera index.
        If no transformation has been found for a camera the identity transformation will be returned
        """
        return self.transformations
    
    def get_result_pointcloud_full(self) -> cwipc_wrapper:
        """Return the resulting point cloud (with each camera mapped by its matrix)"""
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

    
class MultiCameraCoarseColorTarget(MultiCameraCoarse):
    """Do coarse point cloud alignment interactively. The user is presented with a 3D view of each point cloud
    and should select the 4 points of the marker."""

    def __init__(self):
        MultiCameraCoarse.__init__(self)
        self.known_marker_positions = {
            9999: [
                (-0.105, 0, +0.148),   # topleft, blue
                (+0.105, 0, +0.148),   # topright, red
                (+0.105, 0, -0.148),  # botright, yellow
                (-0.105, 0, -0.148),  # botleft, pink
            ]
        }
        self.prompt = "Select blue, red, yellow and pink corners (in that order) in 3D. The press ESC."
    
    def _find_markers(self, pc : open3d.geometry.PointCloud) -> MarkerPositions:
        """Return a dictionary of all markers found in the point cloud (indexed by marker ID, which is always 9999).
        The markers are "found" by having the user select the points in 3D space.
        """
        indices = o3d_pick_points(self.prompt, pc, from000=True)
        points = []
        for i in indices:
            point = pc.points[i]
            if self.debug:
                print(f"find_marker: corner: {point}")
            points.append(point)
        rv = { 9999 : points}
        return rv
    
class MultiCameraCoarseAruco(MultiCameraCoarse):
    """Do coarse point cloud alignment using Aruco markers. The user is presented with a 3D view of each point cloud
    and should orient it so the Aruco makers are visible. The rest is automatic."""

    ARUCO_PARAMETERS = cv2.aruco.DetectorParameters()
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMETERS)

    def __init__(self):
        MultiCameraCoarse.__init__(self)
        # The Aruco is about 14x14cm. Initially, we only know the 3D position of marker with ID 0.
        self.known_marker_positions = {
            0 : [
                (+0.087, 0, +0.087),   # topright, red
                (-0.087, 0, +0.087),   # topleft, blue
                (-0.087, 0, -0.087),  # botleft, pink
                (+0.087, 0, -0.087),  # botright, yellow
            ]
        }
        self.prompt = "Ensure aruco markers are visible. Then type ESC."
    
    def _find_markers(self, pc : open3d.geometry.PointCloud) -> MarkerPositions:
        """Return a dictionary of all markers found in the point cloud (indexed by marker ID)
        The markers are found by mapping the point cloud to a color image and depth image, then finding Aruco markers
        in that color image, then using the depth image to compute the 3D coordinates.
        """

        vis = o3d_show_points(self.prompt, pc, from000=True, keepopen=True)
        o3d_bgr_image_float = vis.capture_screen_float_buffer()
        np_bgr_image_float = np.asarray(o3d_bgr_image_float)
        np_rgb_image_float = np_bgr_image_float[:,:,[2,1,0]]
        np_rgb_image = (np_rgb_image_float * 255).astype(np.uint8)
        areas_2d, ids = self._find_aruco_in_image(np_rgb_image)
        rv : MarkerPositions = {}
        if not ids is None:
            viewControl = vis.get_view_control()
            pinholeCamera = viewControl.convert_to_pinhole_camera_parameters()
            o3d_depth_image_float = vis.capture_depth_float_buffer()
            areas_3d = self._deproject(ids, areas_2d, pinholeCamera, o3d_depth_image_float)
            for i in range(len(ids)):
                id = ids[i]
                area_3d = areas_3d[i]
                rv[id] =  area_3d
        vis.destroy_window()
        return rv
    
    def _deproject(self, ids : Sequence[int], areas_2d : List[List[Sequence[float]]], pinholeCamera, o3d_depth_image_float) -> List[MarkerPosition]:
        # First get the camera parameters
        o3d_extrinsic = pinholeCamera.extrinsic
        o3d_intrinsic = pinholeCamera.intrinsic
        depth_scale = 1
        fx, fy = o3d_intrinsic.get_focal_length()
        cx, cy = o3d_intrinsic.get_principal_point()
        if self.debug:
            print(f"deproject: depth_scale={depth_scale} c={cx},{cy} f={fx},{fy}")
        # Now get the depth image
        np_depth_image_float = np.asarray(o3d_depth_image_float)
        height, width = np_depth_image_float.shape
        min_depth = np.min(np_depth_image_float)
        max_depth = np.max(np_depth_image_float)
        if self.debug:
            print(f"deproject: depth range {min_depth} to {max_depth}, width={width}, height={height}")
        areas_3d : List[List[Tuple[float, float, float]]] = []  # Will be filled with 3D areas
        boxes : List[Any] = [] # Will be filled with open3d boxes (if we want to display them)
        if self.debug:
            # Show the depth-only pointcloud for visual inspection.
            # We move the recangular bound box a little bit away, and we also show
            # in true 3D coorinates where the actual marker is.
            np_depth_image_float = np.asarray(o3d_depth_image_float)
        for area_2d in areas_2d:
            npoints = len(area_2d)
            assert npoints == 4
            orig_transform = np.asarray(o3d_extrinsic)
            transform = transformation_invert(orig_transform)
            if self.debug:
                print(f"deproject: transform={transform}")
            # We want the bounding box (in the D image) for debugging
            assert len(area_2d) == 4
            min_u = max_u = int(area_2d[0][0])
            min_v = max_v = int(area_2d[0][1])
            area_3d : List[Tuple[float, float, float]] = [] # Will be filled with the corners
            for corner_2d in area_2d:
                u, v = corner_2d
                # opencv uses y-down, open3d uses y-up. So convert the v value
                # v = height - v
                u = int(u)
                v = int(v)
                #
                # Update the orthogonal bounding box (for debug display)
                #
                if u < min_u: min_u = u
                if u > max_u: max_u = u
                if v < min_v: min_v = v
                if v > max_v: max_v = v
                #
                # Now find the depth value for this corner.
                #
                d = np_depth_image_float[v, u]
                d = float(d)
                if self.debug:
                    print(f"deproject: corner: u={u}, v={v}, d={d} in 2D space")
                #
                # Convert from u, v, d to x, y, z using intrinsics
                #
                z = d / depth_scale
                x = (u-cx) * z / fx
                y = (v-cy) * z / fy
                #
                # Project using the extrinsics to convert to correct x, y, z coordinates
                #
                np_point = np.array([x, y, z, 1])
                if self.debug:
                    print(f"deproject: corner: x={x}, y={y}, z={z} in 3D camera space (pt={np_point})")
                
                np_point_transformed = (transform @ np_point)

                px = float(np_point_transformed[0])
                py = float(np_point_transformed[1])
                pz = float(np_point_transformed[2])
                fourth = float(np_point_transformed[3])
                if self.debug:
                    print(f"deproject: corner: x={px}, y={py}, z={pz}, fourth={fourth} in 3D pointcloud space (pt={np_point_transformed})")
                area_3d.append((px, py, pz))
            #
            # Found all the corners. Remember them as a 3d area
            #
            areas_3d.append(area_3d)
            #
            # If we need it for display, slightly offset the depth for the bounding box,
            # and compute the geomtry of the actual marker (in 3D)
            #
            if self.debug:
                # We move the recangular bound box a little bit away, and we also show
                # in true 3D coorinates where the actual marker is.
                for v in range(min_v, max_v):
                    for u in range(min_u, max_u):
                        np_depth_image_float[v, u] *= 1.2
                # Create the marker box
                box = open3d.geometry.LineSet()
                box.points = open3d.utility.Vector3dVector(area_3d)
                box.lines = open3d.utility.Vector2iVector([[0,1], [1,2], [2,3], [3, 0], [0, 2], [1, 3]])
                color = [0.4, 0.4, 0]
                box.colors = open3d.utility.Vector3dVector([color, color, color, color, color, color])
                boxes.append(box)
        #
        # All boxes hapve been mapped to 3D.
        #
        # Display the point cloud with the all the rectangles for the markers found
        if self.debug:
            cropped_img = open3d.geometry.Image(np_depth_image_float)
            o3dpc = open3d.geometry.PointCloud.create_from_depth_image(cropped_img, o3d_intrinsic, o3d_extrinsic, depth_scale=1.0)
            tmpvis = open3d.visualization.Visualizer() # type: ignore
            tmpvis.create_window(window_name="Detected markers in 3D space, ESC to close")
            # Add the depth point cloud, with the bounding box cutout
            tmpvis.add_geometry(o3dpc)
            for box in boxes:
                tmpvis.add_geometry(box)
            # Add a coordinate frame
            axes = open3d.geometry.TriangleMesh.create_coordinate_frame()
            tmpvis.add_geometry(axes)
            # Set camera position to where the physical camera was
            tmpViewControl = tmpvis.get_view_control()
            pinholeCamera = tmpViewControl.convert_to_pinhole_camera_parameters()
            pinholeCamera.extrinsic = transformation_identity()
            tmpViewControl.convert_from_pinhole_camera_parameters(pinholeCamera)

            tmpvis.run()
            tmpvis.destroy_window()
        return areas_3d
    
    def _find_aruco_in_image(self, img : cv2.typing.MatLike) -> Tuple[List[List[float]], List[int]]:
        corners, ids, rejected  = self.ARUCO_DETECTOR.detectMarkers(img)
        if self.debug:
            print("find_aruco_in_image: corners:", corners)
            print("find_aruco_in_image: ids:", ids)
        if self.debug:
            outputImage = img.copy()
            cv2.aruco.drawDetectedMarkers(outputImage, corners, ids)
            cv2.imshow("Detected markers in 2D image. ESC to close.", outputImage)
            while True:
                ch = cv2.waitKey()
                if ch == 27:
                    break
                print(f"ignoring key {ch}")
        return corners[0].tolist(), list(ids[0])
