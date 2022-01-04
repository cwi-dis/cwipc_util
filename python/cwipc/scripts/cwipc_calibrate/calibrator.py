import sys
import os
import numpy as np
import open3d
import pprint
import math
import cwipc

from .pointcloud import Pointcloud
from .cameraconfig import CameraConfig, DEFAULT_FILENAME
from .ui import UI
DEBUG=False

FRONTAL_MATRIX = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0,  1.0],
    [0.0, 0.0, -1.0,  0.60],
    [0, 0, 0, 1]
]    
class Calibrator:
    def __init__(self, refpoints):
        self.ui = UI()
        self.cameraserial = []
        self.cameraconfig = None
        self.near = 0
        self.far = 0
        self.height_min = 0
        self.height_max = 0
        self.refpoints = refpoints
        self.grabber = None
        self.cameraserial = []
        self.pointclouds = []
        self.coarse_calibrated_pointclouds = []
        self.fine_calibrated_pointclouds = []
        self.refpointcloud = None
        self.coarse_matrix = []
        self.fine_matrix = []
        self.workdir = os.getcwd() # open3d visualizer can change directory??!?
        sys.stdout.flush()
        
    def __del__(self):
        self.grabber = None
        self.pointclouds = None
        self.coarse_calibrated_pointclouds = None
        self.refpointcloud = None
        
    def setheight(self, height_min, height_max):
        self.height_min = height_min
        self.height_max = height_max
        
    def setdepth(self, near, far):
        self.near = near
        self.far = far
        
    def open(self, grabber, clean, reuse):
        if clean:
            if os.path.exists(DEFAULT_FILENAME):
                os.unlink(DEFAULT_FILENAME)
        elif reuse:
            pass
        elif os.path.exists(DEFAULT_FILENAME):
            self.ui.show_error('%s: cameraconfig.xml already exists, please supply --clean or --reuse argument' % sys.argv[0])
            sys.exit(1)
        self.grabber = grabber
        ok = self.grabber.open()
        if not ok:
            return False
        self.cameraserial = self.grabber.getserials()
        self.cameraconfig = CameraConfig(DEFAULT_FILENAME, read=False)
        self.cameraconfig.copyFrom(self.grabber.cameraconfig)
        return True

    def issynthetic(self):
        return self.grabber.getserials()[0] == "synthetic"
        
    def auto(self):
        self.skip_coarse()
        self.skip_fine()
        # If we have a single camera and no matrix we apply the frontal 1m matrix
        if len(self.cameraserial) == 0:
            pass
        elif len(self.cameraserial) == 1:
            if self.grabber.getmatrix(0) == [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]:
                print("* Single camera setup, assume horizontal frontal camera")
                self.coarse_matrix[0] = FRONTAL_MATRIX
                if self.near == 0 and self.far == 0:
                    print("* Assuming near and far of 40cm and 1.2m")
                    self.near = 0.4
                    self.far = 1.2
                if self.height_min == 0 and self.height_max == 0:
                    print("* Assuming height of 70cm to 170cm")
                    self.height_min = 0.7
                    self.height_max = 1.7
            # Otherwise presume the matrix has already been set
        else:
            if self.grabber.getmatrix(0) == [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]:
                # Uninitialized multi-camera setup. Cannot do automatically
                self.ui.show_error('%s: multi-camera setup, please run calibrator manually' % sys.argv[0])
                sys.exit(1)
        
    def grab(self, noinspect):
        if not self.cameraserial:
            self.ui.show_error('* No realsense cameras found')
            return False
        self.ui.show_message('* Grabbing pointclouds')
        self.get_pointclouds()
        if DEBUG:
            for i in range(len(self.pointclouds)):
                self.ui.show_message('Saving pointcloud {} to file'.format(i))
                self.pointclouds[i].save('pc-%d.ply' % i, cwipc.CWIPC_FLAGS_BINARY)
        #
        # First show the pointclouds for visual inspection.
        #
        if noinspect: return
        grab_ok = False
        while not grab_ok:
            sys.stdout.flush()
            for i in range(len(self.pointclouds)):
                self.ui.show_prompt(f'Showing grabbed pointcloud from camera {i} for visual inspection')
                self.ui.show_points(f'Inspect grab from {self.cameraserial[i]}', self.pointclouds[i], from000=True)
            grab_ok = self.ui.show_question('Can you select the reference points on the alignment target from this pointcloud?', canretry=True)
            if not grab_ok:
                self.ui.show_message('* discarding 10 pointclouds')
                for i in range(10):
                    self.grabber.getpointcloud()
                self.ui.show_message('* Grabbing pointclouds again')
                self.pointclouds = []
                self.get_pointclouds()
        
        
    def run_coarse(self):
        self.ui.show_prompt('Pick reference points on alignment target reference', isedit=True)
        #
        # Pick reference points
        #
        refpoints = self.ui.pick_points('Pick points on reference', self.refpointcloud)
        
        #
        # Pick points in images
        #
        for i in range(len(self.pointclouds)):
            matrix_ok = False
            while not matrix_ok:
                self.ui.show_prompt(f'Pick red, orange, yellow, blue points on camera {i} pointcloud', isedit=True)
                pc_refpoints = self.ui.pick_points(f'Pick points on {self.cameraserial[i]}', self.pointclouds[i], from000=True)
                info = self.align_pair(self.pointclouds[i], pc_refpoints, self.refpointcloud, refpoints, False)
                self.ui.show_prompt(f'Inspect resultant orientation of camera {i} pointcloud')
                new_pc = self.pointclouds[i].transform(info)
                self.ui.show_points(f'Result from {self.cameraserial[i]}', new_pc)
                matrix_ok = self.ui.show_question('Does that look good?', canretry=True)
            assert len(self.coarse_matrix) == i
            assert len(self.coarse_calibrated_pointclouds) == i
            self.coarse_matrix.append(info)
            self.coarse_calibrated_pointclouds.append(new_pc)
        #
        # Show result
        #
        self.ui.show_prompt('Inspect the resultant merged pointclouds of all cameras')
        joined = Pointcloud.from_join(self.coarse_calibrated_pointclouds)
        os.chdir(self.workdir)
        joined.save('cwipc_calibrate_coarse.ply', cwipc.CWIPC_FLAGS_BINARY)
        self.ui.show_points('Inspect manual calibration result', joined)
        
    def skip_coarse(self):
        self.coarse_calibrated_pointclouds = self.pointclouds
        for i in range(len(self.cameraserial)):
            self.coarse_matrix.append([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        
    def apply_bbox(self, bbox):
        if bbox:
            # Apply bounding box to pointclouds
            for i in range(len(self.coarse_calibrated_pointclouds)):
                self.coarse_calibrated_pointclouds[i] = self.coarse_calibrated_pointclouds[i].bbox(bbox)
        joined = Pointcloud.from_join(self.coarse_calibrated_pointclouds)
        self.ui.show_prompt('Inspect pointcloud after applying bounding box')
        self.ui.show_points('Inspect bounding box result', joined)
    
    def run_fine(self, correspondance_dist, inspect):
        print("# Starting fine alignment")
        
        camPositions = []
        #Get positions of all camera origins in world coordinates
        for i in range(0, len(self.coarse_calibrated_pointclouds)):
            matrix = self.grabber.getmatrix(i)
            npMatrix = np.matrix(self.coarse_matrix[i]) @ np.matrix(matrix)
            camVector = npMatrix @ np.array([0, 0, 0, 1])
            camVector = np.array([camVector[0,0],camVector[0,1], camVector[0,2]])
            camPositions.append(camVector)
         
        compute_align_fine = True
        while compute_align_fine:
            self.fine_matrix = []
            self.fine_calibrated_pointclouds = []
            for i in range(len(self.cameraserial)):
                self.fine_matrix.append([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1],
                ])
            if len(self.coarse_calibrated_pointclouds) <= 1:
                self.ui.show_message('* Skipping fine-grained calibration: only one camera')
                self.fine_calibrated_pointclouds = self.coarse_calibrated_pointclouds
                return
        
            print("* Select alignment algorithm:\n\t1) - cumulative ICP using point2point\n\t2) - cumulative ICP using point2plane\n\t3) - pairwise ICP using point2point\n\t4) - pairwise ICP using point2plane\n\t5) - pairwise recursive colored ICP\n\t6) - cumulative multiscale ICP")
            method = sys.stdin.readline().strip().lower()
            
            if (method == '1') or (method == '2') or (method == '6'): #cumulative
                #Not giving good results. shoud we remove color correction?
                #color_correction = True #Enables/disables extra step of alignment based on color
                # print("* Apply color correction? (y, n)")
                # cc = sys.stdin.readline().strip().lower()
                # if cc == 'n':
                    # color_correction = False
                if method == '1':
                    print("## Computing alignment using cumulative ICP point2point:")
                    self.fine_matrix = self.align_fine_cumulative_point2point(self.coarse_calibrated_pointclouds, camPositions, correspondance_dist)
                if method == '2':
                    print("## Computing alignment using cumulative ICP point2plane:")
                    self.fine_matrix = self.align_fine_cumulative_point2plane(self.coarse_calibrated_pointclouds, camPositions, correspondance_dist)
                if method == '6':
                    print("## Computing alignment using cumulative multiscale ICP:")
                    self.fine_matrix = self.align_fine_multiscale_ICP(self.coarse_calibrated_pointclouds, camPositions, correspondance_dist)
                for i in range(len(camPositions)):
                    transformMatrix = self.fine_matrix[i]
                    pc = self.coarse_calibrated_pointclouds[i].clean()
                    transformedPointcloud = pc.transform(transformMatrix)
                    self.fine_calibrated_pointclouds.append(transformedPointcloud)
            else:
                refPointcloud = self.coarse_calibrated_pointclouds[0].clean()
                self.fine_calibrated_pointclouds.append(refPointcloud)
                #Store the order of cameras being aligned
                camIndex = []
                #First camera is used to initialize
                camIndex.append(0)
                
                if method == '3' or method == '4':
                    if method == '3':
                        print("* Choose number of iterations of ICP algorithm (Default 2000) :")
                    if method == '4':
                        print("* Choose number of iterations of ICP algorithm (Default 50) :")
                    iter = int(sys.stdin.readline().strip().lower())
                
                #Loop till all camera clouds are fine aligned
                while (len(self.fine_calibrated_pointclouds) < len(self.coarse_calibrated_pointclouds)):
                    #We now compare the dot product of all (fine) unaligned cameras with all (fine) aligned cameras to find the nearest camera pair
                    #we assume that this will provide the optimale oplossing for overlap of points for ICP to work with
                    dotProducts = np.empty((len(self.fine_calibrated_pointclouds),len(self.coarse_calibrated_pointclouds)))
                    for i in range(0,len(self.coarse_calibrated_pointclouds)):
                        if i not in camIndex:
                            for j in range(0,len(camIndex)):
                                if i == camIndex[j]:
                                    #Arbitrary high negative number so we ignore cameras that are already (fine) aligned
                                    dotProducts[j][i] = -5
                                else:
                                    dotProducts[j][i] = np.dot(camPositions[camIndex[j]],camPositions[i])
                        else:
                            for j in range(0,len(camIndex)):
                                #Arbitrary high negative number so we ignore cameras that are already (fine) aligned
                                dotProducts[j][i] = -5
                    Idx = np.unravel_index(dotProducts.argmax(),dotProducts.shape)
                    ref_cam = camIndex[Idx[0]]
                    src_cam = Idx[1]
                    print(f' -Now calibrating camera {src_cam} to fine align with {ref_cam}')
                    refPointcloud = self.coarse_calibrated_pointclouds[ref_cam].clean()
                    srcPointcloud = self.coarse_calibrated_pointclouds[src_cam].clean()
                    if method == '3':
                        print("## Computing alignment using pairwise ICP point2point:")
                        initMatrix = self.align_fine_point2point(refPointcloud, srcPointcloud, correspondance_dist, [camPositions[ref_cam],camPositions[src_cam]], iter)
                    elif method == '4':
                        print("## Computing alignment using pairwise ICP point2plane:")
                        initMatrix = self.align_fine_point2plane(refPointcloud, srcPointcloud, correspondance_dist, [camPositions[ref_cam],camPositions[src_cam]], iter)
                    else: #method == '5':
                        print("## Computing alignment using pairwise ICP colored:")
                        initMatrix = self.align_fine_rec_colored_ICP(refPointcloud, srcPointcloud, [camPositions[ref_cam], camPositions[src_cam]])
                    transformMatrix = initMatrix @ self.fine_matrix[ref_cam]
                    transformPointcloud = srcPointcloud.transform(transformMatrix)
                    self.fine_calibrated_pointclouds.append(transformPointcloud)
                    self.fine_matrix.append(transformMatrix)
                    camIndex.append(src_cam)
                    newMatrix = transformMatrix
                    newPointcloud = transformPointcloud
                    if inspect:
                        print(f'Fine matrix for camera {self.cameraserial[i]} is:')
                        pprint.pprint(newMatrix)
                        showPCref = refPointcloud.colored((255, 0, 0))
                        showPCsrc = srcPointcloud.colored((0, 255, 0))
                        showPCdst = newPointcloud.colored((0, 0, 255))
                        joined = Pointcloud.from_join((showPCref, showPCsrc, showPCdst))
                        self.ui.show_prompt(f"Inspect alignment of {self.cameraserial[i]} (before: green, after: blue) to reference {self.cameraserial[0]} (red) ")
                        self.ui.show_points('Inspect alignment result', joined)
                #Reorder based on original camera orders
                self.fine_calibrated_pointclouds = [self.fine_calibrated_pointclouds[i] for i in camIndex]
                self.fine_matrix = [self.fine_matrix[i] for i in camIndex]
            #Display results
            self.ui.show_prompt('Inspect the resultant merged pointclouds of all cameras')
            joined = Pointcloud.from_join(self.fine_calibrated_pointclouds)
            os.chdir(self.workdir)
            self.ui.show_points('Inspect fine calibration result', joined)
            print("* Do you like the calibration? (y => continue, n => try another)")
            retry = sys.stdin.readline().strip().lower()
            if retry == 'y':
                compute_align_fine = False
        
        joined.save('cwipc_calibrate_calibrated.ply', cwipc.CWIPC_FLAGS_BINARY)
        print("Result saved as cwipc_calibrate_calibrated.ply")
        
    def skip_fine(self):
        for i in range(len(self.cameraserial)):
            self.fine_matrix.append([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ])
        self.fine_calibrated_pointclouds = self.coarse_calibrated_pointclouds
        
    def save(self):
        # Open3D Visualiser changes directory (!!?!), so change it back
        os.chdir(self.workdir)
        self.writeconfig()
        self.cleanup()
        
    def cleanup(self):
        self.pointclouds = []
        self.coarse_calibrated_pointclouds = []
        self.fine_calibrated_pointclouds = []
        self.refpointcloud = None
        self.grabber = None
        
    def get_pointclouds(self):
        # Create the canonical pointcloud, which determines the eventual coordinate system
        self.refpointcloud = Pointcloud.from_points(self.refpoints)
        # Get the number of cameras and their tile numbers
        maxtile = self.grabber.getcount()
        if DEBUG: print('maxtile', maxtile)
        # Grab one combined pointcloud and split it into tiles
        for i in range(10):
            pc = self.grabber.getpointcloud()
            self.pointclouds = pc.split()
            if len(self.pointclouds) == maxtile: break
            self.ui.show_error(f'Warning: got {len(self.pointclouds)} pointclouds in stead of {maxtile}. Retry.')
        if len(self.pointclouds) != maxtile:
            exit(1)
        #
        # Save captured pointcloud (for possible use later)
        #
        joined = Pointcloud.from_join(self.pointclouds)
        joined.save('cwipc_calibrate_captured.ply', cwipc.CWIPC_FLAGS_BINARY)
    
    def writeconfig(self):
        allcaminfo = ""
        for i in range(len(self.cameraserial)):
            serial = self.cameraserial[i]
            # Find matrix by applying what we found after the matrix read from the original config file (or the identity matrix)
            matrix = self.grabber.getmatrix(i)
            npMatrix = np.matrix(self.fine_matrix[i]) @ np.matrix(self.coarse_matrix[i]) @ np.matrix(matrix)
            matrix = npMatrix.tolist()
            self.cameraconfig.setmatrix(i, matrix)
        if self.near or self.far:
            self.cameraconfig.setdistance(self.near, self.far)
        if self.height_min or self.height_max:
            self.cameraconfig.setheight(self.height_min, self.height_max)
        self.cameraconfig.save()

    def align_pair(self, source, picked_id_source, target, picked_id_target, extended=False):
        assert(len(picked_id_source)>=3 and len(picked_id_target)>=3)
        assert(len(picked_id_source) == len(picked_id_target))
        corr = np.zeros((len(picked_id_source),2))
        corr[:,0] = picked_id_source
        corr[:,1] = picked_id_target

        p2p = open3d.pipelines.registration.TransformationEstimationPointToPoint()
        trans_init = p2p.compute_transformation(source.get_o3d(), target.get_o3d(),
            open3d.utility.Vector2iVector(corr))
        
        if not extended:
            return trans_init

        threshold = 0.01 # 3cm distance threshold
        reg_p2p = open3d.pipelines.registration.registration_icp(source.get_o3d(), target.get_o3d(), threshold, trans_init,
            open3d.pipelines.registration.TransformationEstimationPointToPoint())
        
        return reg_p2p.transformation
    
    def align_fine_point2point(self, source_pc, target_pc, correspondance_dist, cameras, iter):
        '''ICP alignment based on point2point - at the moment this is giving the best results'''
        print('Align fine:')
        source = source_pc.get_o3d()
        target = target_pc.get_o3d()
        
        print(" 1. Downsampling")
        radius_ds = 0.005
        source_down = source.voxel_down_sample(radius_ds)
        target_down = target.voxel_down_sample(radius_ds)
        
        print(" 2. Computing ICP point2point")
        trans_init = np.identity(4)
        reg_p2p = open3d.pipelines.registration.registration_icp(target_down, source_down, correspondance_dist, trans_init,
            open3d.pipelines.registration.TransformationEstimationPointToPoint(), open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = iter))
        
        return reg_p2p.transformation
    
    def align_fine_point2plane(self, source_pc, target_pc, correspondance_dist, cameras, iter):
        '''ICP alignment based on point2plane'''
        source = source_pc.get_o3d()
        target = target_pc.get_o3d()
        
        print("1. Downsampling")
        radius_ds = 0.005
        source_down = source.voxel_down_sample(radius_ds)
        target_down = target.voxel_down_sample(radius_ds)
        
        print("2. Estimating normals")
        radius_n = 0.03
        source_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius_n, max_nn=30))
        source_down = correct_normals(source_down,cameras[0])
        
        target_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius_n, max_nn=30))
        target_down = correct_normals(target_down,cameras[1])
        
        print("3. Computing ICP point2plane")
        trans_init = np.identity(4)
        reg_point2plane = open3d.pipelines.registration.registration_icp(target_down, source_down, correspondance_dist, trans_init, 
            open3d.pipelines.registration.TransformationEstimationPointToPlane(), open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = iter))
        
        return reg_point2plane.transformation
        
    #XXXShishir colored ICP
    def align_fine_colored(self, source, target, threshold, nn_radius):
        trans_init = np.identity(4)
        #Estimate Normals with max nearest neighbors = 30
        nn_radius = 0.02
        source.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius = nn_radius, max_nn=30))
        target.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius = nn_radius, max_nn=30))
        #XXXShishir ToDo: Check if downsampling is needed
        #Applying colored ICP registration
        reg_c = open3d.pipelines.registration.registration_colored_icp(source, target, nn_radius, trans_init, open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration = 50 ))
        #reg_p2p = open3d.registration.registration_icp(target.get_o3d(), source.get_o3d(), threshold, trans_init, open3d.registration.TransformationEstimationPointToPoint())    
        
        return reg_c.transformation
        
    def align_fine_rec_colored_ICP(self, source, target, cameras):
        '''colored ICP recursive with different voxel radius'''
        source = source.get_o3d()
        target = target.get_o3d()
        voxel_radius = [0.02, 0.01, 0.005]
        max_iter = [50, 30, 10]
        current_transformation = np.identity(4)
        print("3. Colored point cloud registration")
        for scale in range(3):
            iter = max_iter[scale]
            radius = voxel_radius[scale]
            print([iter, radius, scale])

            print("3-1. Downsample with a voxel size %.2f" % radius)
            source_down = source.voxel_down_sample(radius)
            #open3d.visualization.draw_geometries([source_down])
            target_down = target.voxel_down_sample(radius)

            print("3-2. Estimate normal.")
            source_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
            source_down = correct_normals(source_down,cameras[0])
            target_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
            target_down = correct_normals(target_down,cameras[1])
            
            #open3d.visualization.draw_geometries([source_down])
            print("3-3. Applying colored point cloud registration")
            result_icp = open3d.pipelines.registration.registration_colored_icp(
                target_down, source_down, radius, current_transformation,
                open3d.pipelines.registration.TransformationEstimationForColoredICP(),
                open3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,relative_rmse=1e-6,max_iteration=iter))
            current_transformation = result_icp.transformation
            print("DONE")
        
        return current_transformation
    
    def align_fine_cumulative_point2point(self, pointclouds, cam_pos, correspondance_dist):
        '''Given all the point clouds and its camera positions, it computes the transformations for the fine alignment'''
        print("* Choose number of iterations of ICP algorithm (Default 500) :")
        iter = int(sys.stdin.readline().strip().lower())
        cam_order = get_cameras_order(cam_pos)
        pcs = [] #list of ordered pcs
        transformations = [] #list of ordered transformations
        for i in range(len(cam_order)):
            pcs.append(pointclouds[cam_order[i]].clean().get_o3d())
            #print(len(pointclouds[cam_order[i]].get_o3d().points),"->",len(pcs[-1].points))
            transformations.append(np.identity(4))
            
        
            
        radius_ds = 0.005 #voxel downsampling radius
        tpc = pcs[0] #the target pc
        tpc_down = tpc.voxel_down_sample(radius_ds) #downsampled version
        aligned_pc = open3d.geometry.PointCloud()
        
        
        for i in range(1,len(pcs)):
            init_transf = np.identity(4)
            src_pc_down = pcs[i].voxel_down_sample(radius_ds)
            reg_p2p = open3d.pipelines.registration.registration_icp(src_pc_down, tpc_down, correspondance_dist, init_transf,
                open3d.pipelines.registration.TransformationEstimationPointToPoint(), open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = iter))
            transformations[i] = reg_p2p.transformation @ transformations[i]
            tf_pc = pcs[i].transform(reg_p2p.transformation)
            tpc += tf_pc
            tpc_down = tpc.voxel_down_sample(radius_ds)
            aligned_pc += tf_pc
            print("-Aligned tile",i)

        #Extra step to fine align first cam
        aligned_pc_down = aligned_pc.voxel_down_sample(radius_ds)
        init_transf = np.identity(4)
        reg_p2p = open3d.pipelines.registration.registration_icp(pcs[0].voxel_down_sample(radius_ds), aligned_pc_down, correspondance_dist, init_transf,
            open3d.pipelines.registration.TransformationEstimationPointToPoint(), open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = iter))
        transformations[0] = reg_p2p.transformation @ transformations[0]
        tf_pc = pcs[0].transform(reg_p2p.transformation)
        aligned_pc += tf_pc
        print("-Aligned pc",0)

        # if(color_correction): #not working most of the times, should we remove it?
            # print("# Applying correction based on color")
            # pcs_down = [] #list of downsampled point clouds
            # for i in range(len(cam_order)):
                # pc = pcs[i]
                # pc_down = pc.voxel_down_sample(radius_ds)
                # pc_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                # pcs_down.append(correct_normals(pc_down,cam_pos[i]))

            # for i in range(len(pcs_down)):
                # p0 = pcs_down[i]
                # aux = open3d.geometry.PointCloud()
                # for j in range(len(pcs_down)):
                    # if j!= i:
                        # aux = aux + pcs_down[j]
                # init_transf = np.identity(4)
                # result_icp = open3d.registration.registration_colored_icp(p0, aux, correspondance_dist, init_transf,
                    # open3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,relative_rmse=1e-6,max_iteration=20))
                # pcs_down[i] = p0.transform(result_icp.transformation)
                # transformations[i] = result_icp.transformation @ transformations[i]

        final_pc = open3d.geometry.PointCloud()
        tup = [] #tuple list to recover original order
        for i in range(len(cam_order)):
            final_pc += pointclouds[cam_order[i]].clean().get_o3d().transform(transformations[i])
            tup.append((cam_order[i],transformations[i]))
        #open3d.visualization.draw_geometries([final_pc])

        ordered_tup = sorted(tup, key=lambda x: x[0]) #sort tuple (cam_id,transformation) by cam_id to recover original order
        return [i[1] for i in ordered_tup] #return transformations in original order
        
    def align_fine_cumulative_point2plane(self, pointclouds, cam_pos, correspondance_dist):
        '''Given all the point clouds and its camera positions, it computes the transformations for the fine alignment'''
        print("* Choose number of iterations of ICP algorithm (Default 500) :")
        iter = int(sys.stdin.readline().strip().lower())
        cam_order = get_cameras_order(cam_pos)
        pcs = [] #list of ordered pcs
        transformations = [] #list of ordered transformations
        for i in range(len(cam_order)):
            pcs.append(pointclouds[cam_order[i]].clean().get_o3d())
            #print(len(pointclouds[cam_order[i]].get_o3d().points),"->",len(pcs[-1].points))
            transformations.append(np.identity(4))
            
        radius_ds = 0.005 #voxel downsampling radius
        tpc = pcs[0] #the target pc
        tpc_down = tpc.voxel_down_sample(radius_ds) #downsampled version
        aligned_pc = open3d.geometry.PointCloud()
        
        
        for i in range(1,len(pcs)):
            init_transf = np.identity(4)
            src_pc_down = pcs[i].voxel_down_sample(radius_ds)
            
            #estimate normals for Point2Plane
            src_pc_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
            tpc_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
            
            reg_p2p = open3d.pipelines.registration.registration_icp(
                src_pc_down, 
                tpc_down,
                correspondance_dist, init_transf,
                open3d.pipelines.registration.TransformationEstimationPointToPlane(), 
                open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = iter))
            transformations[i] = reg_p2p.transformation @ transformations[i]
            tf_pc = pcs[i].transform(reg_p2p.transformation)
            tpc += tf_pc
            tpc_down = tpc.voxel_down_sample(radius_ds)
            aligned_pc += tf_pc
            print("-Aligned pc",i)

        #Extra step to fine align first cam
        aligned_pc_down = aligned_pc.voxel_down_sample(radius_ds)
        init_transf = np.identity(4)
        
        #estimate normals for Point2Plane
        src = pcs[0].voxel_down_sample(radius_ds)
        src.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        tgt = aligned_pc_down
        tgt.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
        
        reg_p2p = open3d.pipelines.registration.registration_icp(
            src, tgt, 
            correspondance_dist, init_transf,
            open3d.pipelines.registration.TransformationEstimationPointToPlane(), open3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = iter))
        transformations[0] = reg_p2p.transformation @ transformations[0]
        tf_pc = pcs[0].transform(reg_p2p.transformation)
        aligned_pc += tf_pc
        print("-Aligned pc",0)

        # if(color_correction): #not working most of the times, should we remove it?
            # print("# Applying correction based on color")
            # pcs_down = [] #list of downsampled point clouds
            # for i in range(len(cam_order)):
                # pc = pcs[i]
                # pc_down = pc.voxel_down_sample(radius_ds)
                # pc_down.estimate_normals(open3d.geometry.KDTreeSearchParamHybrid(radius=0.02, max_nn=30))
                # pcs_down.append(correct_normals(pc_down,cam_pos[i]))

            # for i in range(len(pcs_down)):
                # p0 = pcs_down[i]
                # aux = open3d.geometry.PointCloud()
                # for j in range(len(pcs_down)):
                    # if j!= i:
                        # aux = aux + pcs_down[j]
                # init_transf = np.identity(4)
                # result_icp = open3d.registration.registration_colored_icp(p0, aux, correspondance_dist, init_transf,
                    # open3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,relative_rmse=1e-6,max_iteration=20))
                # pcs_down[i] = p0.transform(result_icp.transformation)
                # transformations[i] = result_icp.transformation @ transformations[i]

        final_pc = open3d.geometry.PointCloud()
        tup = [] #tuple list to recover original order
        for i in range(len(cam_order)):
            final_pc += pointclouds[cam_order[i]].clean().get_o3d().transform(transformations[i])
            tup.append((cam_order[i],transformations[i]))
        #open3d.visualization.draw_geometries([final_pc])

        ordered_tup = sorted(tup, key=lambda x: x[0]) #sort tuple (cam_id,transformation) by cam_id to recover original order
        return [i[1] for i in ordered_tup] #return transformations in original order
        
    def align_fine_multiscale_ICP(self, pointclouds, cam_pos, correspondance_dist):
        '''Given all the point clouds and its camera positions, it computes the transformations for the fine alignment    
            It is based in : http://www.open3d.org/docs/release/tutorial/reconstruction_system/refine_registration.html?highlight=registration_colored_icp#fine-grained-registration but using a cumulative method instead of pairwise.
        '''
        
        print("   Choose ICP method:\n\t1) point2point\n\t2) point2plane\n\t3) color")
        mode = int(sys.stdin.readline().strip().lower())
        cam_order = get_cameras_order(cam_pos)
        print("cam_order = ",cam_order)
        pcs = [] #list of ordered pcs
        transformations = [] #list of ordered transformations
        for i in range(len(cam_order)):
            pcs.append(pointclouds[cam_order[i]].clean().get_o3d())
            #print(len(pointclouds[cam_order[i]].get_o3d().points),"->",len(pcs[-1].points))
            transformations.append(np.identity(4))
            
        tpc = pcs[0] #the target pc
        aligned_pc = open3d.geometry.PointCloud()
        
        for i in range(1,len(pcs)):
            current_transformation = self.multiscale_icp(pcs[i], tpc, correspondance_dist, mode, np.identity(4))
            transformations[i] = current_transformation @ transformations[i]
            tf_pc = pcs[i].transform(current_transformation)
            tpc += tf_pc
            aligned_pc += tf_pc
            print("-Aligned tile",cam_order[i])

        #Extra step to fine align first cam -> better leave first camera static.
        #current_transformation = self.multiscale_icp(pcs[0], aligned_pc, correspondance_dist, mode, np.identity(4))
        #transformations[0] = current_transformation @ transformations[0]
        #tf_pc = pcs[0].transform(current_transformation)
        #aligned_pc += tf_pc
        #print("-Aligned tile",0)

        final_pc = open3d.geometry.PointCloud()
        tup = [] #tuple list to recover original order
        for i in range(len(cam_order)):
            final_pc += pointclouds[cam_order[i]].clean().get_o3d().transform(transformations[i])
            tup.append((cam_order[i],transformations[i]))
        #open3d.visualization.draw_geometries([final_pc])

        ordered_tup = sorted(tup, key=lambda x: x[0]) #sort tuple (cam_id,transformation) by cam_id to recover original order
        return [i[1] for i in ordered_tup] #return transformations in original order
    
    def multiscale_icp(self, source, target, voxel_size, mode, init_transformation=np.identity(4)):
        vox_scale = np.asarray([voxel_size, voxel_size/2.0, voxel_size/4.0]) #original-> vox_scale = [0.04, 0.02, 0.01]
        max_iter = [50, 30, 14]
        current_transformation = init_transformation
        for i, scale in enumerate(range(len(max_iter))):  # multi-scale approach
            iter = max_iter[scale]
            distance_threshold = vox_scale[scale] #voxel_size * 1.4
            print("#   voxel_size {}".format(vox_scale[scale]), "\t",iter,"iterations")
            source_down = source.voxel_down_sample(vox_scale[scale])
            target_down = target.voxel_down_sample(vox_scale[scale])
            if mode == 1: #"point_to_point"
                result_icp = open3d.pipelines.registration.registration_icp(
                    source_down, target_down, distance_threshold,
                    current_transformation,
                    open3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    open3d.pipelines.registration.ICPConvergenceCriteria(
                        max_iteration=iter))
            else:
                source_down.estimate_normals(
                    open3d.geometry.KDTreeSearchParamHybrid(radius=vox_scale[scale] *
                                                         2.0,
                                                         max_nn=30))
                target_down.estimate_normals(
                    open3d.geometry.KDTreeSearchParamHybrid(radius=vox_scale[scale] *
                                                         2.0,
                                                         max_nn=30))
                if mode == 2: #"point_to_plane"
                    result_icp = open3d.pipelines.registration.registration_icp(
                        source_down, target_down, distance_threshold,
                        current_transformation,
                        open3d.pipelines.registration.
                        TransformationEstimationPointToPlane(),
                        open3d.pipelines.registration.ICPConvergenceCriteria(
                            max_iteration=iter))
                if mode == 3: #"color"
                    result_icp = open3d.pipelines.registration.registration_colored_icp(
                        source_down, target_down, vox_scale[scale],
                        current_transformation,
                        open3d.pipelines.registration.
                        TransformationEstimationForColoredICP(),
                        open3d.pipelines.registration.ICPConvergenceCriteria(
                            relative_fitness=1e-6,
                            relative_rmse=1e-6,
                            max_iteration=iter))
            current_transformation = result_icp.transformation
        
        return current_transformation
    
def unit_vector(vector):
    ''' Returns the unit vector of the vector.  '''
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    ''' Returns the signed angle in radians between vectors 'v1' and 'v2'::
            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
    '''
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    angle = np.arccos(np.minimum(1, np.dot(v1_u, v2_u))) #np.minimum is used because sometimes dot product was out of the interval [-1,1] and was throwing errors.
    if(v1_u[0]*v2_u[2] - v1_u[2]*v2_u[0] > 0):
        angle = -angle;
    return angle

def correct_normals(pc,cameraPos):
        '''corrects the direction of the normals based on the camera position'''
        points = np.asarray(pc.points)
        normals = np.asarray(pc.normals)
        
        count = 0
        for k in range(len(points)):
            v1 = cameraPos - points[k]
            v2 = normals[k]
            #print("v1:",v1,"v2:",v2)
            #flip de normal vector if it is not pointing towards the sensor:
            angle = angle_between(v1,v2)
            if angle > math.pi/2 or angle < -math.pi/2:
                normals[k][0] = -normals[k][0]
                normals[k][1] = -normals[k][1]
                normals[k][2] = -normals[k][2]
                count += 1
        print("- Corrected",count,"normals")
        pc.normals = open3d.utility.Vector3dVector(normals)
        
        return pc
    
def get_cameras_order(camPositions):
    '''Given the camera positions it returns the circular order of the cameras'''
    centroid = [0.0,0.0,0.0]
    for i in range(len(camPositions)):
        centroid += camPositions[i]
    centroid /= len(camPositions)
    v_0 = camPositions[0]-centroid
    tup = []
    for i in range(len(camPositions)):
        v1 = camPositions[i]-centroid
        angle = math.degrees(angle_between(v_0,v1))
        tup.append((i,angle))
    ordered_tup = sorted(tup, key=lambda x: x[1]) #sort tuple (id,angle) by angle
    return [i[0] for i in ordered_tup]
