import cwipc
import numpy as np
import open3d
import copy
from typing import Optional, Any, Tuple, List
from ... import cwipc_wrapper

O3dPointcloud = Any

class Pointcloud:
    """A class that handles both cwipc pointclouds and o3d pointclouds and converts between them"""
    cwipc : Optional[cwipc_wrapper]
    o3d: O3dPointcloud

    def __init__(self):
        self.cwipc = None
        self.o3d = None
        
    def __del__(self):
        if self.cwipc:
            self.cwipc.free()
        self.cwipc = None
        self.o3d = None
        
    def _ensure_cwipc(self) -> None:
        """Internal - make sure the cwipc is valid"""
        if self.cwipc: return
        tilenum = 1
        points = self.o3d.points # type: ignore
        colors = self.o3d.colors # type: ignore
        tiles = np.array([[tilenum]]*len(points))
        cwiPoints = np.hstack((points, colors, tiles))
        cwiPoints = list(map(lambda p : (p[0],p[1],p[2],int(p[3]*255),int(p[4]*255),int(p[5]*255), int(p[6])), list(cwiPoints)))
        self.cwipc = cwipc.cwipc_from_points(cwiPoints, 0)
        
    def _ensure_o3d(self) -> None:
        """internal - make sure the o3d pc is valid"""
        if self.o3d: return
        # Note that this method is inefficient, it can probably be done
        # in-place with some numpy magic
        assert self.cwipc
        pcpoints = self.cwipc.get_points()
        points = []
        colors = []
        for p in pcpoints:
            points.append([p.x, p.y, p.z])  
            colors.append((float(p.r)/255.0, float(p.g)/255.0, float(p.b)/255.0))
        points_v_np = np.matrix(points)
        points_v = open3d.utility.Vector3dVector(points_v_np)
        colors_v = open3d.utility.Vector3dVector(colors)
        self.o3d = open3d.geometry.PointCloud()
        self.o3d.points = points_v
        self.o3d.colors = colors_v
        
    @classmethod
    def from_cwipc(cls, pc : cwipc_wrapper) -> 'Pointcloud':
        """Create Pointcloud from cwipc"""
        assert pc
        self = cls()
        self.cwipc = pc
        return self

    @classmethod
    def from_o3d(cls, o3d) -> 'Pointcloud':
        """Create Pointcloud from o3d pc"""
        assert o3d
        self = cls()
        self.o3d = o3d
        return self
        
    @classmethod
    def from_points(cls, points) -> 'Pointcloud':
        """Create Pointcloud from list of xyzrgbt tuples"""
        assert points
        self = cls()
        pc = cwipc.cwipc_from_points(points, 0)
        self.cwipc = pc
        return self
        
    @classmethod
    def from_file(cls, filename : str) -> 'Pointcloud':
        """Create Pointcloud from ply file"""
        self = cls()
        pc = cwipc.cwipc_read(filename, 0)
        self.cwipc = pc
        return self
        
    @classmethod
    def from_join(cls, pointclouds : List['Pointcloud']) -> 'Pointcloud':
        """Create tiled Pointcloud from separate pointclouds"""
        allPoints = []
        tileNum = 1
        for src_pc in pointclouds:
            src_cwipc = src_pc.get_cwipc()
            points = src_cwipc.get_points()
            points = map(lambda p : (p.x, p.y, p.z, p.r, p.g, p.b, tileNum), points)
            allPoints += points
            tileNum *= 2
        return cls.from_points(allPoints)
        
    def get_cwipc(self) -> cwipc_wrapper:
        """Return cwipc object"""
        self._ensure_cwipc()
        assert self.cwipc
        return self.cwipc
        
    def get_o3d(self) -> O3dPointcloud:
        """Return o3d pc object"""
        self._ensure_o3d()
        return self.o3d
        
    def save(self, filename : str, flags : int=0) -> None:
        """Save to PLY file"""
        self._ensure_cwipc()
        assert self.cwipc
        cwipc.cwipc_write(filename, self.cwipc, flags)
        
    def split(self) -> List['Pointcloud']:
        """Split into per-tile Pointcloud objects"""
        self._ensure_cwipc()
        assert self.cwipc
        alltiles = set()
        #
        # This is stupid code, because we should really get a list of tilenumbers as parameter.
        # The current code depends on the serial numbers having been collected in tile-number-order.
        #
        for pt in self.cwipc.get_points():
            alltiles.add(pt.tile)
        rv = []
        alltiles = list(alltiles)
        alltiles.sort()
        for tilenum in alltiles:
            tile_pc = cwipc.cwipc_tilefilter(self.cwipc, tilenum)
            assert tile_pc
            rv.append(self.__class__.from_cwipc(tile_pc))
        return rv
        
    def transform(self, matrix : np.matrix) -> 'Pointcloud':
        """Return pointcloud multiplied by a matrix"""
        # Note that this method is inefficient, it can probably be done
        # in-place with some numpy magic
        self._ensure_cwipc()
        assert self.cwipc
        pcpoints = self.cwipc.get_points()
        points = []
        colort = []
        for p in pcpoints:
            points.append([p.x, p.y, p.z])  
            colort.append((p.r, p.g, p.b, p.tile))
        npPoints = np.matrix(points)
        submatrix = matrix[:3, :3]
        translation = matrix[:3, 3]
        npPoints = (submatrix * npPoints.T).T
        npPoints = npPoints + translation
        assert len(npPoints) == len(points)
        newPoints = []
        for i in range(len(points)):
            newPoint = tuple(npPoints[i].tolist()[0]) + colort[i]
            newPoints.append(newPoint)
        return self.__class__.from_points(newPoints)
        
    def bbox(self, bbox : Tuple[float, float, float, float, float, float]) -> 'Pointcloud':
        """Return pointcloud limited to a bounding box"""
        x0, x1, y0, y1, z0, z1 = bbox
        self._ensure_cwipc()
        assert self.cwipc
        pcpoints = self.cwipc.get_points()
        newPoints = []
        for p in pcpoints:
            if p.x < x0 or p.x > x1:
                continue
            if p.y < y0 or p.y > y1:
                continue
            if p.z < z0 or p.z > z1:
                continue
            newPoints.append(p)
        return self.__class__.from_points(newPoints)
        
    def colored(self, rgb : Tuple[int, int, int]) -> 'Pointcloud':
        r, g, b = rgb
        self._ensure_cwipc()
        assert self.cwipc
        pcpoints = self.cwipc.get_points()
        pcpoints = copy.deepcopy(pcpoints)
        for p in pcpoints:
            p.r = r
            p.g = g
            p.b = b
        return self.__class__.from_points(pcpoints)
    
    def clean(self) -> 'Pointcloud':
        """Removes statistical outliers (undesired points). Ex. occlusion tales"""
        clean_pc, index = self.get_o3d().remove_statistical_outlier(nb_neighbors=20,std_ratio=2.0)
        return self.__class__.from_o3d(clean_pc)
    
    def clean_background(self) -> 'Pointcloud':
        #Cleaning green background color
        colors = np.asarray(self.get_o3d().colors)
        npoints = len(colors)
        background_ids = []
        for id in range(0,npoints):
            color = colors[id]
            if (color[1]>color[0]) & (color[1]>color[2]):
                background_ids.append(id)
        pc_clean = self.get_o3d().select_by_index(background_ids,invert=True)
        
        #remove radius outliers
        mean_dist = self.compute_mean_dist_pc()
        pc_out,l = pc_clean.remove_radius_outlier(int(0.02/mean_dist),0.02) # it removes points that do not have int nb_points in a sphere of float radius
        
        return self.__class__.from_o3d(pc_out)
        
    def compute_mean_dist_pc(self) -> float:
        """Computes average distance between points in the pointcloud"""
        pc = self.get_o3d()
        pcd_tree = open3d.geometry.KDTreeFlann(pc)
        n_points = len(pc.points)
        total_dist = 0.0
        for i in range(0,n_points):
            p1 = pc.points[i]
            [k, idx, _] = pcd_tree.search_knn_vector_3d(p1, 2)
            p2 = pc.points[idx[1]]
            dist = float(np.linalg.norm(p2-p1))
            total_dist += dist
        mean_dist = total_dist/n_points
        #print("Mean dist between points:",mean_dist)
        return mean_dist
