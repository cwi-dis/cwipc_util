import sys
import os
import time
import shutil
import argparse
import json
from typing import Optional, List, cast, Tuple
import numpy
import cwipc
from cwipc.net.abstract import *
from ._scriptsupport import *
from cwipc.registration.abstract import *
from cwipc.registration.util import *
import cwipc.registration.coarse
import cwipc.registration.analyze
import cwipc.registration.multicamera

try:
    import cwipc.realsense2
    # Workaround for https://github.com/intel-isl/Open3D/issues/3283
    _ = cwipc.realsense2.cwipc_realsense2_dll_load()
except ModuleNotFoundError:
    cwipc.realsense2 = None
try:
    import cwipc.kinect
except ModuleNotFoundError:
    cwipc.kinect = None


DEFAULT_FILENAME = "cameraconfig.json"

def main():
    parser = ArgumentParser(description="Register cwipc cameras (realsense, kinect, virtual) so they produce overlapping point clouds.")
    def twofloats(s):
        f1, f2 = s.split(',')
        return float(f1), float(f2)
    parser.add_argument("--fromxml", action="store_true", help="Convert cameraconfig.xml to cameraconfig.json and exit")
    
    parser.add_argument("--clean", action="store_true", help=f"Remove old {DEFAULT_FILENAME} and calibrate from scratch")
    parser.add_argument("--nograb", metavar="PLYFILE", action="store", help=f"Don't use grabber but use .ply file grabbed earlier, using {DEFAULT_FILENAME} from same directory.")
    parser.add_argument("--skip", metavar="N", type=int, action="store", help="Skip the first N captures")
    parser.add_argument("--coarse", action="store_true", help="Do coarse calibration (default: only if needed)")
    parser.add_argument("--nofine", action="store_true", help="Don't do fine calibration (default: always do it)")
    parser.add_argument("--noregister", action="store_true", help="Don't do any registration, only create cameraconfig.json if needed")
    parser.add_argument("--debug", action="store_true", help="Produce step-by-step pointclouds and cameraconfigs in directory cwipc_register_debug")
    parser.add_argument("--dry_run", action="store_true", help="Don't modify cameraconfig file")
    parser.add_argument("recording", nargs='?', help="A directory with recordings (realsense or kinect) for which to do registration")
    args = parser.parse_args()
    beginOfRun(args)
    reg = Registrator(args)
    reg.run()


PythonTrafo = List[List[float]]

class Transform:
    """A 4x4 spatial transformation matrix, represented as both a list of lists of floats or a numpy matrix."""
    trafo : PythonTrafo
    matrix : RegistrationTransformation
    changed : bool

    def __init__(self, trafo : PythonTrafo):
        self._set(trafo)
        self.changed = False

    def is_dirty(self) -> bool:
        return self.changed
    
    def reset(self) -> None:
        self.changed = False

    def get(self) -> PythonTrafo:
        return self.trafo
    
    def _set(self, trafo : PythonTrafo) -> None:
        self.trafo = trafo
        self.matrix = transformation_frompython(self.trafo)

    def get_matrix(self) -> RegistrationTransformation:
        return self.matrix
    
    def set_matrix(self, matrix : RegistrationTransformation) -> None:
        if numpy.array_equal(matrix, self.matrix):
            return
        self.matrix = matrix
        self.trafo = transformation_topython(self.matrix)
        self.changed = True

    def is_identity(self) -> bool:
        return numpy.array_equal(self.matrix, transformation_identity())
    
class CameraConfig:
    transforms : List[Transform]

    def __init__(self, filename):
        self.filename = filename
        self.cameraconfig = dict()
        oldconfig = self.filename + '~'
        if os.path.exists(oldconfig):
            os.unlink(oldconfig)
        self.transforms = []
        self._dirty = False

    def init_transforms(self) -> None:
        cameras = self["camera"]
        camera_count = len(cameras)
        self.transforms = [
            Transform(cam["trafo"]) for cam in cameras
        ]

    def camera_count(self) -> int:
        return len(self.transforms)
    
    def is_dirty(self) -> bool:
        if self._dirty:
            return True
        for t in self.transforms:
            if t.is_dirty():
                return True
        return False
    
    def get_transform(self, camnum : int) -> Transform:
        return self.transforms[camnum]
    
    def refresh_transforms(self) -> None:
        """Copy transforms back to cameraconfig"""
        for cam_num in range(len(self.transforms)):
            trafo = self.transforms[cam_num].get()
            self.cameraconfig["camera"][cam_num]["trafo"] = trafo
    
    def is_identity(self) -> bool:
        for t in self.transforms:
            if not t.is_identity():
                return False
        return True

    def load(self, jsondata : bytes) -> None:
        self.cameraconfig = json.loads(jsondata)
        self.init_transforms()
        self._dirty = False
        # Workaround for bug (only in realsense_playback?) 2023-12-30
        if self.cameraconfig["type"] == "":
            self.cameraconfig["type"] = self.cameraconfig["camera"][0]["type"]

    def save(self) -> None:
        # First time keep the original config file
        self.refresh_transforms()
        oldconfig = self.filename + '~'
        if os.path.exists(self.filename) and not os.path.exists(oldconfig):
            os.rename(self.filename, oldconfig)
        json.dump(self.cameraconfig, open(self.filename, 'w'), indent=2)
        # Mark transforms as not being dirty any longer.
        for t in self.transforms:
            t.reset()
        self._dirty = False

    def save_to(self, filename : str) -> None:
        self.refresh_transforms()
        json.dump(self.cameraconfig, open(filename, 'w'), indent=2)

    def get(self) -> bytes:
        return json.dumps(self.cameraconfig).encode('utf8')
    
    def __setitem__(self, key, item):
        self.cameraconfig[key] = item
        self._dirty = True

    def __getitem__(self, key):
        return self.cameraconfig[key]

class Registrator:
    def __init__(self, args : argparse.Namespace):
        self.coarse_aligner_class = cwipc.registration.coarse.MultiCameraCoarseInteractive
        self.fine_aligner_class = cwipc.registration.multicamera.MultiCamera
        self.analyzer_class = cwipc.registration.analyze.RegistrationAnalyzer
        self.args = args
        self.verbose = self.args.verbose
        self.debug = self.args.debug
        self.dry_run = self.args.dry_run
        if self.args.recording:
            if self.args.cameraconfig:
                print("Cannot use --cameraconfig with a recording")
                sys.exit(1)
            self.args.cameraconfig = os.path.realpath(os.path.join(self.args.recording, DEFAULT_FILENAME))
        if not self.args.cameraconfig:
            self.args.cameraconfig = DEFAULT_FILENAME
        self.cameraconfig = CameraConfig(self.args.cameraconfig)
        self.progname = sys.argv[0]
        self.capturerFactory : Optional[cwipc_tiledsource_factory_abstract] = None
        self.capturerName = None
        self.capturer = None
        if self.debug:
            if os.path.exists("cwipc_register_debug"):
                shutil.rmtree("cwipc_register_debug")
            os.mkdir("cwipc_register_debug")
            print(f"Will produce debug files in cwipc_register_debug")

    def prompt(self, message : str):
        print(f"{message}")
        
    def run(self) -> bool:
        if self.args.recording:
            # Special case: register a recording. First make sure we actually have a
            # cameraconfig file, otherwise generate it.
            if not self.initialize_recording():
                return False
        self.args.nodecode = True
        self.capturerFactory, self.capturerName = cwipc_genericsource_factory(self.args)
        if self.args.fromxml:
            # Special case: load XML config file name create JSON config file
            self.json_from_xml()
            return True
        if not self.open_capturer():
            print("Cannot open capturer. Presume missing cameraconfig, try to create it.")
            self.create_cameraconfig()
            if not self.open_capturer():
                print("Still cannot open capturer. Giving up.")
                return False
        # Get initial cameraconfig and save it
        assert self.capturer
        self.cameraconfig.load(self.capturer.get_config())
        if not self.dry_run:
            self.cameraconfig.save()
        if self.args.noregister:
            return True
        if self.args.coarse or self.cameraconfig.is_identity():
            new_pc = None
            while new_pc == None:
                self.prompt("Coarse calibration: capturing aruco/color target")
                pc = self.capture()
                if self.debug:
                    self.save_pc(pc, "step1_capture_coarse")
                new_pc = self.coarse_calibration(pc)
                pc.free()
                pc = None
            assert new_pc
            if self.debug:
                self.save_pc(new_pc, "step2_after_coarse")
            new_pc.free()
            new_pc = None
            if not self.dry_run:
                self.cameraconfig.save()
        if not self.args.nofine:
            self.prompt("Fine calibration: capturing human-sized object")
            pc = self.capture()
            if self.debug:
                self.save_pc(pc, "step3_capture_fine")
            new_pc = self.fine_calibration(pc)
            pc.free()
            pc = None
            if self.debug:
                self.save_pc(new_pc, "step4_after_fine")
            new_pc.free()
            new_pc = None
            if not self.dry_run:
                self.cameraconfig.save()
        
        return False
    
    def initialize_recording(self) -> bool:
        if os.path.exists(self.args.cameraconfig):
            return True
        kinect_files = []
        realsense_files = []
        for fn in os.listdir(self.args.recording):
            if fn.lower().endswith(".mkv"):
                kinect_files.append(fn)
            if fn.lower().endswith(".bag"):
                realsense_files.append(fn)
        if kinect_files and realsense_files:
            print(f"Directory {self.args.recording} contains both .mkv and .bag files")
            return False
        if not kinect_files and not realsense_files:
            print(f"Directory {self.args.recording} contains neither .mkv nor .bag files")
            return False
        camtype = "kinect_offline"
        if realsense_files:
            camtype = "realsense_playback"
        allfiles = kinect_files + realsense_files
        # Create the camera definitions
        camera = [
            dict(filename=fn, type=camtype)
                for fn in allfiles
        ]
        cameraconfig = dict(
            version=3,
            type=camtype,
            system=dict(),
            postprocessing=dict(
                depthfilterparameters=dict(

                )
            ),
            camera=camera
        )
        json.dump(cameraconfig, open(self.args.cameraconfig, "w"), indent=4)
        if self.verbose:
            print(f"Created {self.args.cameraconfig}")
        return True
    
    def json_from_xml(self):
        capturer = capturerFactory("cameraconfig.xml") # type: ignore
        json_data = capturer.get_config()
        open(self.args.cameraconfig, 'wb').write(json_data)

    def open_capturer(self) -> bool:
        assert self.capturerFactory
        # xxxjack this will eventually fail for generic capturer
        if not self.capturerName:
            print(f"{self.progname}: selected capturer does not need calibration")
            return False
        # Step one: Try to open with an existing cameraconfig.
        try:
            self.capturer = self.capturerFactory()
        except cwipc.CwipcError:
            return False
        return True
    
    def create_cameraconfig(self) -> None:
        """Attempt to open a capturer without cameraconfig file. Then save the empty cameraconfig."""
        tmpFactory, _ = cwipc_genericsource_factory(self.args, autoConfig=True)
        tmpCapturer = tmpFactory()
        self.cameraconfig.load(tmpCapturer.get_config())
        if not self.dry_run:
            self.cameraconfig.save()
            if self.verbose:
                print(f"Saved {self.args.cameraconfig}")
        else:
            print(f"Not saving {self.args.cameraconfig}, --dry-run specified.")
        


    def capture(self) -> cwipc.cwipc_wrapper:
        if self.args.nograb:
            pc = cwipc.cwipc_read(self.args.nograb, 0)
            return pc
        assert self.capturer
        if self.args.skip:
            for i in range(self.args.skip):
                ok = self.capturer.available(True)
                if ok:
                    pc = self.capturer.get()
                    if pc != None:
                        pc.free()
        ok = self.capturer.available(True)
        assert ok
        pc = self.capturer.get()
        assert pc
        assert pc.count() > 0
        return cast(cwipc.cwipc_wrapper, pc)
            
    def save_pc(self, pc : cwipc_wrapper, label : str) -> None:
        if self.debug:
            filename = f"cwipc_register_debug/{label}-pointcloud.ply"
            cwipc.cwipc_write(filename, pc)
            filename = f"cwipc_register_debug/{label}-cameraconfig.json"
            self.cameraconfig.save_to(filename)
            print(f"Saved pointcloud and cameraconfig for {label}")
            

    def coarse_calibration(self, pc : cwipc_wrapper) -> Optional[cwipc_wrapper]:
        aligner = self.coarse_aligner_class()
        aligner.add_tiled_pointcloud(pc)
        start_time = time.time()
        ok = aligner.run()
        stop_time = time.time()
        if self.verbose:
            print(f"coarse aligner ran for {stop_time-start_time:.3f} seconds")
        if not ok:
            print("Could not do coarse registration")
            return None
        # Get the resulting transformations, and store them in cameraconfig.
        transformations = aligner.get_result_transformations()
        for cam_num in range(len(transformations)):
            matrix = transformations[cam_num]
            t = self.cameraconfig.get_transform(cam_num)
            t.set_matrix(matrix)
        # Get the newly aligned pointcloud to test for alignment, and return it
        new_pc = aligner.get_result_pointcloud_full()
        correspondence, _ = self.check_alignment(new_pc, 0, "coarse calibration")
        self.cameraconfig["correspondence"] = correspondence
        return new_pc

    def fine_calibration(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        aligner = self.fine_aligner_class()
        # This number sets a threashold for the best possible alignment.
        # xxxjack it should be computed from the source point clouds
        original_capture_precision = 0.001

        aligner.verbose = True
        aligner.show_plot = False
        aligner.add_tiled_pointcloud(pc)
        for cam_index in range(self.cameraconfig.camera_count()):
            aligner.set_original_transform(cam_index, self.cameraconfig.get_transform(cam_index).get_matrix())
        start_time = time.time()
        ok = aligner.run()
        stop_time = time.time()
        if self.verbose:
            print(f"fine aligner ran for {stop_time-start_time:.3f} seconds")
        if not ok:
            print("Could not do fine registration")
            sys.exit(1)
        # Get the resulting transformations, and store them in cameraconfig.
        transformations = aligner.get_result_transformations()
        for cam_num in range(len(transformations)):
            matrix = transformations[cam_num]
            t = self.cameraconfig.get_transform(cam_num)
            t.set_matrix(matrix)
        # Get the newly aligned pointcloud to test for alignment, and return it
        new_pc = aligner.get_result_pointcloud_full()
        correspondence, _ = self.check_alignment(new_pc, 0, "fine calibration")
        correspondence, _ = self.check_alignment(new_pc, 0, "coarse calibration")
        self.cameraconfig["correspondence"] = correspondence
        return new_pc

    def check_alignment(self, pc : cwipc_wrapper, original_capture_precision : float, label : str) -> Tuple[float, int]:
        analyzer = cwipc.registration.analyze.RegistrationAnalyzer()
        analyzer.add_tiled_pointcloud(pc)
        analyzer.label = label
        start_time = time.time()
        analyzer.run()
        stop_time = time.time()
        print(f"analyzer ran for {stop_time-start_time:.3f} seconds")
        results = analyzer.get_ordered_results()
        print(f"Sorted correspondences after {label}")
        worst_correspondence = 0
        for camnum, correspondence, weight in results:
            print(f"\tcamnum={camnum}, correspondence={correspondence}, weight={weight}")
            if correspondence > worst_correspondence:
                worst_correspondence = correspondence

        camnum_to_fix = None
        correspondence = results[0][1]
        for i in range(len(results)):
            if results[i][1] >= original_capture_precision:
                camnum_to_fix = results[i][0]
                correspondence = results[i][1]
                break
        if self.debug:
            analyzer.plot(filename="", show=True)
        assert camnum_to_fix
        return worst_correspondence, camnum_to_fix      

    
if __name__ == '__main__':
    main()
    