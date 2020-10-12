import cwipc
import numpy as np
import open3d
import copy

class Pointcloud:
    """A class that handles both cwipc pointclouds and o3d pointclouds and converts between them"""
    
    def __init__(self):
        self.cwipc = None
        self.o3d = None
        
    def __del__(self):
        if self.cwipc:
            self.cwipc.free()
        self.cwipc = None
        self.o3d = None
        
    def _ensure_cwipc(self):
        """Internal - make sure the cwipc is valid"""
        if self.cwipc: return
        tilenum = 1
        points = self.o3d.points
        colors = self.o3d.colors
        tiles = np.array([[tilenum]]*len(points))
        cwiPoints = np.hstack((points, colors, tiles))
        cwiPoints = list(map(lambda p : (p[0],p[1],p[2],int(p[3]*255),int(p[4]*255),int(p[5]*255), int(p[6])), list(cwiPoints)))
        self.cwipc = cwipc.cwipc_from_points(cwiPoints, 0)
        
    def _ensure_o3d(self):
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
    def from_cwipc(klass, pc):
        """Create Pointcloud from cwipc"""
        self = klass()
        self.cwipc = pc
        return self

    @classmethod
    def from_o3d(klass, o3d):
        """Create Pointcloud from o3d pc"""
        self = klass()
        self.o3d = o3d
        return self
        
    @classmethod
    def from_points(klass, points):
        """Create Pointcloud from list of xyzrgbt tuples"""
        self = klass()
        pc = cwipc.cwipc_from_points(points, 0)
        self.cwipc = pc
        return self
        
    @classmethod
    def from_file(klass, filename):
        """Create Pointcloud from ply file"""
        self = klass()
        pc = cwipc.cwipc_read(filename)
        self.cwipc = pc
        return self
        
    @classmethod
    def from_join(klass, pointclouds):
        """Create tiled Pointcloud from separate pointclouds"""
        allPoints = []
        tileNum = 1
        for src_pc in pointclouds:
            src_cwipc = src_pc.get_cwipc()
            points = src_cwipc.get_points()
            points = map(lambda p : (p.x, p.y, p.z, p.r, p.g, p.b, tileNum), points)
            allPoints += points
            tileNum *= 2
        return klass.from_points(allPoints)
        
    def get_cwipc(self):
        """Return cwipc object"""
        self._ensure_cwipc()
        return self.cwipc
        
    def get_o3d(self):
        """Return o3d pc object"""
        self._ensure_o3d()
        return self.o3d
        
    def save(self, filename):
        """Save to PLY file"""
        self._ensure_cwipc()
        cwipc.cwipc_write(filename, self.cwipc)
        
    def split(self):
        """Split into per-tile Pointcloud objects"""
        self._ensure_cwipc()
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
            rv.append(self.__class__.from_cwipc(tile_pc))
        return rv
        
    def transform(self, matrix):
        """Return pointcloud multiplied by a matrix"""
        # Note that this method is inefficient, it can probably be done
        # in-place with some numpy magic
        self._ensure_cwipc()
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
        
    def bbox(self, bbox):
        """Return pointcloud limited to a bounding box"""
        x0, x1, y0, y1, z0, z1 = bbox
        self._ensure_cwipc()
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
        
    def colored(self, rgb):
        r, g, b = rgb
        self._ensure_cwipc()
        pcpoints = self.cwipc.get_points()
        pcpoints = copy.deepcopy(pcpoints)
        for p in pcpoints:
            p.r = r
            p.g = g
            p.b = b
        return self.__class__.from_points(pcpoints)
    
    def clean_background(self):
        #Cleaning green background color
        colors = np.asarray(self.get_o3d().colors)
        npoints = len(colors)
        background_ids = []
        for id in range(0,npoints):
            color = colors[id]
            if (color[1]>color[0]) & (color[1]>color[2]):
                background_ids.append(id)
        pc_clean = self.get_o3d().select_by_index(background_ids,invert=True)
        #pc_clean = self.get_o3d()
        pc_out,l = pc_clean.remove_radius_outlier(44,0.02) #remove_radius_outlier(self, int nb_points, float radius)
        
        return self.__class__.from_o3d(pc_out)
        
