
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
        self.serial_for_tilenum : Dict[int, str] = {}
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
        count = len(self.per_camera_tilenum)
        assert count > 0 # Otherwise this has been called too early
        return count
    
    def set_serial_dict(self, sd : Dict[int, str]) -> None:
        self.serial_for_tilenum = sd

    def tilenum_for_camera_index(self, cam_index : int) -> int:
        """Returns the tilenumber (used in the point cloud) for this index (used in the results)"""
        return self.per_camera_tilenum[cam_index]

    def camera_index_for_tilenum(self, tilenum : int) -> int:
        """Returns the  index (used in the results) for this tilenumber (used in the point cloud)"""
        for i in range(len(self.per_camera_tilenum)):
            if self.per_camera_tilenum[i] == tilenum:
                return i
        assert False, f"Tilenum {tilenum} not known"

    def _init_transformations(self) -> None:
        # Initialize transformations if not already done
        if self.transformations == []:
            for i in range(self.camera_count()):
                self.transformations.append(transformation_identity())

    def set_transformation(self, tilenum : int, trafo : RegistrationTransformation) -> None:
        self._init_transformations()
        self.transformations[tilenum] = trafo
    
    def _get_unregistered_tiles(self) -> List[int]:
        """Return list of all tilenum that still have an identity transform (and therefore need to be fixed)"""
        rv = []
        self._init_transformations()
        identity = transformation_identity()
        for i in range(len(self.transformations)):
            if (self.transformations[i] == identity).all():
                rv.append(i)
        return rv
    
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
        self._init_transformations()
        assert len(tilenums) == len(self.per_camera_o3d_pointclouds)
        assert len(tilenums) == len(self.per_camera_tilenum)
        assert len(tilenums) == len(self.transformations)

    def run(self) -> bool:
        """Run the algorithm"""
        assert self.original_pointcloud
        # Initialize the analyzer
        self._prepare()
        # Find the markers in each of the pointclouds
        ok = True
        self._find_markers_all_tiles()
        
        assert self.known_marker_positions

        another_pass_wanted = True
        while another_pass_wanted:
            another_pass_wanted = False # Will be set to True if we find any new information, which will help another pass
            tilenums_to_register = self._get_unregistered_tiles()
            if self.verbose:
                print(f"coarse: attempting to register tiles {tilenums_to_register}")
            # But there are some tiles not in tilenums_to_register that we still want to make a pass over, 
            # because in the previous pass they were registered, and they may have information on markers we have not seen yet.
            tilenums_to_register = range(len(self.per_camera_o3d_pointclouds))
            for tilenum in tilenums_to_register:
                camnum = self.tilenum_for_camera_index(tilenum)
                o3d_pc = self.per_camera_o3d_pointclouds[tilenum]
                this_tile_markers = self.markers[tilenum]
                for id, area in this_tile_markers.items():
                    if not self._check_marker(area):
                        continue
                    if id in self.known_marker_positions:
                        #
                        # This is a marker we know. Align it.
                        #
                        wanted_marker_corners = self.known_marker_positions[id]
                        this_marker_corners = area
                        this_transform = self._align_marker(camnum, o3d_pc, wanted_marker_corners, this_marker_corners)
                        if this_transform is None:
                            continue
                        old_transform = self.transformations[tilenum]
                        had_transform = not (old_transform == transformation_identity()).all() # Can happen if we found one for a previous marker
                        if had_transform:
                            # See how much they differ
                            delta_mat = np.abs(this_transform - old_transform)
                            delta = np.add.reduce(delta_mat, None)
                            if self.verbose:
                                print(f"coarse: tile {tilenum}: marker {id}: new registration matrix differs {delta} from old one")
                        else:
                            if self.verbose:
                                print(f"coarse: tile {tilenum}: marker {id}: created transformation matrix")
                            self.transformations[tilenum] = this_transform
                    else:
                        if self.verbose:
                            print(f"coarse: tile {tilenum}: marker {id}: unknown marker found")
                        # If we have a transformation we can compute the 3D position of this new marker
                        tile_transform = self.transformations[tilenum]
                        tile_transform_valid = not (tile_transform == transformation_identity()).all()
                        if tile_transform_valid:
                            # Good! We have found a marker we didn't know about.
                            # Convert the camera-local coordinates to world coordinates
                            new_area : MarkerPosition = []
                            for cam_point in area:
                                x = float(cam_point[0])
                                y = float(cam_point[1])
                                z = float(cam_point[2])
                                np_point = np.array([x, y, z, 1])
                                np_point_transformed = (tile_transform @ np_point)
                                x = float(np_point_transformed[0])
                                y = float(np_point_transformed[1])
                                z = float(np_point_transformed[2])
                                new_area.append((x, y, z))
                            if self.verbose:
                                print(f"coarse: tile {tilenum}: marker {id}: corners {new_area}")
                            self.known_marker_positions[id] = new_area
                            # We want to do another round over all tiles, maybe this marker helps adjustment.
                            another_pass_wanted = True
        return self._get_unregistered_tiles() == []

    def _find_markers_all_tiles(self) -> None:
        self.markers = []
        for idx in range(len(self.per_camera_o3d_pointclouds)):
            o3d_pc = self.per_camera_o3d_pointclouds[idx]
            camnum = self.per_camera_tilenum[idx]
            markers = self._find_markers(0, idx)
            if self.verbose:
                print(f"find_markers_all_tiles: camera={camnum}: {markers}q")
            self.markers.append(markers)
        assert len(self.per_camera_o3d_pointclouds) == len(self.markers)
        
    def _check_marker(self, marker : MarkerPosition) -> bool:
        """Return False if the MarkerPosition cannot be a valid marker"""
        if len(marker) == 4:
            return True
        print(f"Error: marker has {len(marker)} corners in stead of 4")
        return False
    
    def _find_markers(self, passnum : int, idx : int) -> MarkerPositions:
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
    
    def _find_markers(self, passnum : int, idx : int) -> MarkerPositions:
        """Return a dictionary of all markers found in the point cloud (indexed by marker ID, which is always 9999).
        The markers are "found" by having the user select the points in 3D space.
        """
        o3dpc = self.per_camera_o3d_pointclouds[idx]
        tilenum = self.per_camera_tilenum[idx]
        indices = o3d_pick_points(f"Cam {tilenum}: {self.prompt}", o3dpc, from000=True)
        points = []
        for i in indices:
            point = o3dpc.points[i]
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
    
    def _find_markers(self, passnum : int, idx : int) -> MarkerPositions:
        """Return a dictionary of all markers found in the point cloud (indexed by marker ID)
        The markers are found by mapping the point cloud to a color image and depth image, then finding Aruco markers
        in that color image, then using the depth image to compute the 3D coordinates.
        """
        o3dpc = self.per_camera_o3d_pointclouds[idx]
        tilenum = self.per_camera_tilenum[idx]
            
        vis = o3d_show_points(f"Pass {passnum} tile {tilenum}: Ensure markers are visible. ESC to close.", o3dpc, from000=True, keepopen=True)
        o3d_bgr_image_float = vis.capture_screen_float_buffer()
        np_bgr_image_float = np.asarray(o3d_bgr_image_float)
        np_rgb_image_float = np_bgr_image_float[:,:,[2,1,0]]
        np_rgb_image = (np_rgb_image_float * 255).astype(np.uint8)
        areas_2d, ids = self._find_aruco_in_image(passnum, tilenum, np_rgb_image)
        rv : MarkerPositions = {}
        if not ids is None:
            viewControl = vis.get_view_control()
            pinholeCamera = viewControl.convert_to_pinhole_camera_parameters()
            o3d_depth_image_float = vis.capture_depth_float_buffer()
            areas_3d = self._deproject(passnum, tilenum, ids, areas_2d, pinholeCamera, o3d_depth_image_float)
            for i in range(len(ids)):
                id = ids[i]
                area_3d = areas_3d[i]
                rv[id] =  area_3d
        vis.destroy_window()
        return rv
    
    def _deproject(self, passnum : int, camnum : int, ids : Sequence[int], areas_2d : List[List[Sequence[float]]], pinholeCamera, o3d_depth_image_float) -> List[MarkerPosition]:
        show_depth_map = self.debug

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
        if show_depth_map:
            # Show the depth-only pointcloud for visual inspection.
            # We move the recangular bound box a little bit away, and we also show
            # in true 3D coorinates where the actual marker is.
            np_depth_image_float = np.asarray(o3d_depth_image_float)
        for idx in range(len(ids)):
            #for area_2d in areas_2d:
            area_2d = areas_2d[idx]
            if self.debug:
                print(f"deproject: examine marker {idx} of {len(ids)}, id={ids[idx]}, area={area_2d}")
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
            if show_depth_map:
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
        if show_depth_map:
            cropped_img = open3d.geometry.Image(np_depth_image_float)
            o3dpc = open3d.geometry.PointCloud.create_from_depth_image(cropped_img, o3d_intrinsic, o3d_extrinsic, depth_scale=1.0)
            tmpvis = open3d.visualization.Visualizer() # type: ignore
            tmpvis.create_window(window_name=f"Pass {passnum} cam {camnum}: 3D depthmap of markers found. ESC to close.")
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
    
    def _find_aruco_in_image(self, passnum : int, tilenum : int, img : cv2.typing.MatLike) -> Tuple[List[List[List[float]]], List[int]]:
        show_aruco_results = self.verbose
        corners, ids, rejected  = self.ARUCO_DETECTOR.detectMarkers(img)
        if self.debug:
            print("find_aruco_in_image: corners:", corners)
            print("find_aruco_in_image: ids:", ids)
        if show_aruco_results:
            outputImage = img.copy()
            cv2.aruco.drawDetectedMarkers(outputImage, corners, ids)
            winTitle = f"pass {passnum} cam {tilenum}: Detected markers in 2D image. ESC to close."
            cv2.imshow(winTitle, outputImage)
            while True:
                ch = cv2.waitKey()
                if ch == 27:
                    break
                print(f"ignoring key {ch}")
            cv2.destroyWindow(winTitle)
        # Th way the aruco detector returns information is weird. Sometimes it is a single numpy matrix, sometimes a list or tuple of vectors...
        rv_corners : List[List[List[float]]] = []
        rv_ids : List[int] = []
        if ids != None:
            for i in range(len(ids)):
                rv_ids.append(int(ids[i]))
                area = corners[i]
                if area.shape == (1, 4, 2):
                    area = area[0]
                rv_corners.append(area.tolist())

        return rv_corners, rv_ids

class MultiCameraCoarseArucoRgb(MultiCameraCoarseAruco):

    def _find_markers(self, passnum : int, idx : int) -> MarkerPositions:
        """Return a dictionary of all markers found in the point cloud (indexed by marker ID)"""
        tilenum = self.per_camera_tilenum[idx]
        np_rgb_image, np_depth_image = self._get_rgb_depth_images(tilenum)
        if np_rgb_image is None or np_depth_image is None:
            print(f"Warning: RGB or Depth image not captured. Revert to interactive image capture.")
            return MultiCameraCoarseAruco._find_markers(self, passnum, idx)
        areas_2d, ids = self._find_aruco_in_image(passnum, tilenum, np_rgb_image)
        rv : MarkerPositions = {}
        print(f"xxxjack areas_2d: {areas_2d}, ids {ids}")
        for idx in range(len(ids)):
            area_2d = areas_2d[idx]
            if self.debug:
                print(f"find_markers: examine marker {idx} of {len(ids)}, id={ids[idx]}, area={area_2d}")
            npoints = len(area_2d)
            assert npoints == 4
            for corner_2d in area_2d:
                u, v = corner_2d
                # opencv uses y-down, open3d uses y-up. So convert the v value
                # v = height - v
                u = int(u)
                v = int(v)
                d = self._get_depth_value(np_depth_image, u, v)
                print(f"find_markers: u,v,d={(u, v, d)}")
        return rv
    
    def _get_depth_value(self, np_depth_image : cv2.typing.MatLike, x : int, y : int) -> int:
        """Return the depth value at (x, y), possibly searching around if the specific depth value is missing"""
        rv = int(np_depth_image[y, x])
        if rv != 0:
            return rv
        if self.verbose:
            print(f"Warning: need to interpolate depth value for {(x, y)}")
        return 0
    
    def _get_rgb_depth_images(self, tilenum : int) -> Tuple[Optional[cv2.typing.MatLike], Optional[cv2.typing.MatLike]]:
        """Return the RGB and Depth images for a given tile, from the point cloud auxiliary data."""
        serial = self.serial_for_tilenum.get(tilenum)
        if not serial:
            print(f"getrgb_depth_images: Unknown tilenum {tilenum}, no serial number known")
            return None, None
        assert self.original_pointcloud
        auxdata = self.original_pointcloud.access_auxiliary_data()
        if not auxdata or auxdata.count() == 0:
            print(f"get_rgb_depth_images: tilenum {tilenum}: no auxdata")
            assert 0
            return None, None
        depth_image : Optional[cv2.typing.MatLike] = None
        rgb_image : Optional[cv2.typing.MatLike] = None
        for aux_index in range(auxdata.count()):
            if auxdata.name(aux_index) == "rgb." + serial:
                assert rgb_image == None
                depth_descr = self._parse_aux_description(auxdata.description(aux_index))
                depth_width = depth_descr['width']
                image_height = depth_descr['height']
                depth_stride = depth_descr['stride']
                depth_bpp = 0
                if 'bpp' in depth_descr:
                    depth_bpp = depth_descr['bpp']
                elif 'format' in depth_descr:
                    depth_format = depth_descr['format']
                    if depth_format == 2:
                        depth_bpp = 3 # RGB
                    elif depth_format == 3:
                        depth_bpp = 4 # RGBA
                    elif depth_format == 4:
                        depth_bpp = 2 # 16-bit grey
                    else:
                        assert False, "Unknown format in RGB auxdata format specifier"
                assert depth_bpp in (3, 4)
                depth_data = auxdata.data(aux_index)
                np_image_data_bytes = np.array(depth_data)
                np_image_data = np.reshape(np_image_data_bytes, (image_height, depth_width, depth_bpp))
                # Select B, G, R channels
                np_image_data = np_image_data[:,:,[2,1,0]]
                rgb_image = np_image_data
            if auxdata.name(aux_index) == "depth." + serial:
                assert depth_image == None
                depth_descr = self._parse_aux_description(auxdata.description(aux_index))
                depth_width = depth_descr['width']
                depth_height = depth_descr['height']
                depth_stride = depth_descr['stride']
                depth_bpp = 0
                if 'bpp' in depth_descr:
                    depth_bpp = depth_descr['bpp']
                elif 'format' in depth_descr:
                    depth_format = depth_descr['format']
                    if depth_format == 2:
                        depth_bpp = 3 # RGB
                    elif depth_format == 3:
                        depth_bpp = 4 # RGBA
                    elif depth_format == 4:
                        depth_bpp = 2 # 16-bit grey
                    else:
                        assert False, "Unknown format in Depth auxdata format specifier"
                assert depth_bpp ==2
                depth_data = auxdata.data(aux_index)
                np_depth_data_bytes = np.array(depth_data)
                np_depth_data = np.reshape(np_depth_data_bytes, (depth_height, depth_width, depth_bpp))
                np_depth_lower = np_depth_data[:,:,0]
                np_depth_upper = np_depth_data[:,:,1]
                np_depth_data = np_depth_lower + (np_depth_upper * 256)
                depth_image = np_depth_data
        return rgb_image, depth_image
         
    def _parse_aux_description(self, description : str) -> Dict[str, Any]:
        """Helper method: parse an auxdata description string"""
        rv = {}
        fields = description.split(',')
        for f in fields:
            k, v = f.split('=')
            try:
                v = int(v)
            except ValueError:
                pass
            rv[k] = v
        return rv
    