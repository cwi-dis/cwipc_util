
import copy
import math
from typing import List, Optional, Any, Tuple, Sequence, cast
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

MarkerPosition = Any

class MultiCameraCoarse(MultiAlignmentAlgorithm):
    """Align multiple cameras.
    """

    def __init__(self):
        self.debug = False
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
            marker_pos = self._find_marker(o3d_pc)
            while not self._check_marker(marker_pos):
                print(f"Please try again")
                marker_pos = self._find_marker(o3d_pc)
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
            if self.verbose:
                print(f"_align_marker: cam {camnum}: rmse={rmse}")
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
    """Do coarse point cloud alignment interactively. The user is presented with a 3D view of each point cloud
    and should select the 4 points of the marker"""

    def __init__(self):
        MultiCameraCoarse.__init__(self)
        self.wanted_marker = [
            [-0.105, 0, +0.148],   # topleft, blue
            [+0.105, 0, +0.148],   # topright, red
            [+0.105, 0, -0.148],  # botright, yellow
            [-0.105, 0, -0.148],  # botleft, pink
        ]
        self.prompt = "Select blue, red, yellow and pink corners (in that order) in 3D. The press ESC."

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
            if self.debug:
                print(f"find_marker: corner: {point}")
            points.append(point)
        rv = points
        return rv
    
class MultiCameraCoarsePointcloud(MultiCameraCoarse):

    ARUCO_PARAMETERS = cv2.aruco.DetectorParameters()
    ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    ARUCO_DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMETERS)

    def __init__(self):
        MultiCameraCoarse.__init__(self)
        # The Aruco is about 14x14cm
        self.wanted_marker = [
            [+0.07, 0, +0.07],   # topright, red
            [-0.07, 0, +0.07],   # topleft, blue
            [-0.07, 0, -0.07],  # botleft, pink
            [+0.07, 0, -0.07],  # botright, yellow
        ]
        self.prompt = "Ensure aruco markers are visible. Then type ESC."

    def _check_marker(self, marker : Optional[MarkerPosition]) -> bool:
        if marker is None:
            if self.verbose:
                print("check_marker: Error: no marker found")
            return False
        if len(marker) == 4:
            return True
        if self.verbose:
            print(f"check_marker: Error: marker has {len(marker)} points, expected 4")
        return False
    
    def _find_marker(self, pc : open3d.geometry.PointCloud) -> Optional[MarkerPosition]:
        vis = o3d_show_points(self.prompt, pc, from000=True, keepopen=True)
        o3d_bgr_image_float = vis.capture_screen_float_buffer()
        np_bgr_image_float = np.asarray(o3d_bgr_image_float)
        np_rgb_image_float = np_bgr_image_float[:,:,[2,1,0]]
        np_rgb_image = (np_rgb_image_float * 255).astype(np.uint8)
        areas, ids = self._find_aruco_in_image(np_rgb_image)
        rv = None
        if ids != None:
            viewControl = vis.get_view_control()
            pinholeCamera = viewControl.convert_to_pinhole_camera_parameters()
            o3d_depth_image_float = vis.capture_depth_float_buffer()
            for i in range(len(ids)):
                if ids[i] == 0:
                    corners = areas[i][0]
                    points = self._deproject(corners, pinholeCamera, o3d_depth_image_float)
                    rv = points
                    break
        vis.destroy_window()
        return rv
    
    def _deproject(self, corners : cv2.Mat, pinholeCamera, o3d_depth_image_float) -> Optional[MarkerPosition]:
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
        rv = []
        orig_transform = np.asarray(o3d_extrinsic)
        transform = transformation_invert(orig_transform)
        if self.debug:
            print(f"deproject: transform={transform}")
        # We want the bounding box (in the D image) for debugging
        assert corners.any()
        min_u = max_u = int(corners[0][0])
        min_v = max_v = int(corners[0][1])

        for pt in corners:
            u, v = pt
            # opencv uses y-down, open3d uses y-up. So convert the v value
            # v = height - v
            u = int(u)
            v = int(v)
            if u < min_u: min_u = u
            if u > max_u: max_u = u
            if v < min_v: min_v = v
            if v > max_v: max_v = v
            d = np_depth_image_float[v, u]
            d = float(d)
            if self.debug:
                print(f"deproject: corner: u={u}, v={v}, d={d} in 2D space")
            
            z = d / depth_scale
            x = (u-cx) * z / fx
            y = (v-cy) * z / fy
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
            rv.append((px, py, pz))
        # Display the point cloud with the rectangle found
        if self.debug:
            # Show the depth-only pointcloud for visual inspection.
            # We move the recangular bound box a little bit away, and we also show
            # in true 3D coorinates where the actual marker is.
            np_depth_image_float = np.asarray(o3d_depth_image_float)
            w, h = np_depth_image_float.shape
            for v in range(min_v, max_v):
                for u in range(min_u, max_u):
                    np_depth_image_float[v, u] *= 1.2
            cropped_img = open3d.geometry.Image(np_depth_image_float)
            o3dpc = open3d.geometry.PointCloud.create_from_depth_image(cropped_img, o3d_intrinsic, o3d_extrinsic, depth_scale=1.0)
            tmpvis = open3d.visualization.Visualizer() # type: ignore
            tmpvis.create_window(window_name="Detected markers in 3D space, ESC to close")
            # Add the depth point cloud, with the bounding box cutout
            tmpvis.add_geometry(o3dpc)
            # Add the marker box
            box = open3d.geometry.LineSet()
            box.points = open3d.utility.Vector3dVector(rv)
            box.lines = open3d.utility.Vector2iVector([[0,1], [1,2], [2,3], [3, 0], [0, 2], [1, 3]])
            color = [0.4, 0.4, 0]
            box.colors = open3d.utility.Vector3dVector([color, color, color, color, color, color])
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
        return rv
    
    def _find_aruco_in_image(self, img : cv2.typing.MatLike) -> Tuple[Sequence[cv2.typing.MatLike], cv2.typing.MatLike]:
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
        return corners, ids
