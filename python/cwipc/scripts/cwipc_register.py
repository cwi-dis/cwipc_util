import sys
import os
import argparse
import json
from typing import Optional, List, cast
import numpy
import cwipc
from cwipc.net.abstract import *
from ._scriptsupport import *
from cwipc.registration.abstract import *
from cwipc.registration.util import *

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
    parser.add_argument("--coarse", action="store_true", help="Do coarse calibration (default: only if needed)")
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
        self.init_transforms()

    def init_transforms(self) -> None:
        cameras = self["camera"]
        camera_count = len(cameras)
        self.transforms = [
            Transform(cam["trafo"]) for cam in cameras
        ]

    def camera_count(self) -> int:
        return len(self.transforms)
    
    def is_dirty(self) -> bool:
        for t in self.transforms:
            if t.is_dirty():
                return True
        return False
    
    def get_transform(self, camnum : int) -> Transform:
        return self.transforms[camnum]
    
    def is_identity(self) -> bool:
        for t in self.transforms:
            if not t.is_identity():
                return False
        return True

    def load(self, jsondata : bytes) -> None:
        self.cameraconfig = json.loads(jsondata)

    def save(self) -> None:
        # First time keep the original config file
        oldconfig = self.filename + '~'
        if os.path.exists(self.filename) and not os.path.exists(oldconfig):
            os.rename(self.filename, oldconfig)
        json.dump(self.cameraconfig, open(self.filename, 'w'))

    def get(self) -> bytes:
        return json.dumps(self.cameraconfig).encode('utf8')
    
    def __setitem__(self, key, item):
        self.cameraconfig[key] = item

    def __getitem__(self, key):
        return self.cameraconfig[key]

class Registrator:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.verbose = self.args.verbose
        if not self.args.cameraconfig:
            self.args.cameraconfig = DEFAULT_FILENAME
        self.cameraconfig = CameraConfig(self.args.cameraconfig)
        self.progname = sys.argv[0]
        self.capturerFactory : Optional[cwipc_tiledsource_factory_abstract] = None
        self.capturerName = None
        self.capturer = None

    def prompt(self, message : str):
        print(f"{message}")
        
    def run(self) -> bool:
        self.args.nodecode = True
        self.capturerFactory, self.capturerName = cwipc_genericsource_factory(self.args)
        if self.args.fromxml:
            # Special case: load XML config file name create JSON config file
            self.json_from_xml()
            return True
        if not self.open_capturer():
            return False
        # Get initial cameraconfig and save it
        assert self.capturer
        self.cameraconfig.load(self.capturer.get_config())
        print(f"xxxjack cameraconfig {self.cameraconfig.get()}")
        self.cameraconfig.save()
        if self.args.coarse or self.cameraconfig.is_identity():
            self.prompt("Coarse calibration: capturing aruco/color target")
            pc = self.capture()
            if self.args.debug:
                self.save_pc(pc, "step1_capture_coarse")
            new_pc = self.coarse_calibration(pc)
            if self.args.debug:
                self.save_pc(new_pc, "step2_after_coarse")
        if self.args.fine:
            self.prompt("Fine calibration: capturing human-sized object")
            pc = self.capture()
            if self.args.debug:
                self.save_pc(pc, "step3_capture_fine")
            new_pc = self.fine_calibration(pc)
            if self.args.debug:
                self.save_pc(new_pc, "step4_after_fine")
    
        return False
    
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
        self.capturer = self.capturerFactory()
        return True
    
    def capture(self) -> cwipc.cwipc_wrapper:
        if self.args.nograb:
            pc = cwipc.cwipc_read(self.args.nograb, 0)
            return pc
        else:
            assert self.capturer
            for i in range(len(10)):
                ok = self.capturer.available(True)
                assert ok
                pc = self.capturer.get()
                if pc != None and pc.count() != 0:
                    return cast(cwipc.cwipc_wrapper, pc)
        assert False       
            
    def save_pc(self, pc : cwipc_wrapper, label : str) -> None:
        # xxxjack
        pass

    def coarse_calibration(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        # xxxjack
        return pc

    def fine_calibration(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        # xxxjack
        return pc

def foo():
    refpoints = targets[args.target]["points"]
    prog = Calibrator(refpoints)
    if args.nograb:
        grabber = FileGrabber(args.nograb)
    else:
        grabber = LiveGrabber(capturerFactory)
    noInspect = args.nograb
    try:
    
        ok = prog.open(grabber, clean=args.clean, reuse=(args.reuse or args.auto))
        if not ok:
            # Being unable to open the grabber is not an error for --auto
            if args.auto:
                sys.exit(0)
            else:
                sys.exit(1)
        
        if args.auto:
            prog.auto()
        elif args.coarse:
            prog.grab(noInspect)
            prog.run_coarse()
            prog.skip_fine()
        elif args.fine:
            prog.grab(noInspect)
            prog.skip_coarse()
            prog.run_fine(args.corr, args.finspect)
        else:
            print(f"{sys.argv[0]}: Specify one of --auto, --coarse, --fine")
            sys.exit(1)

        prog.save()
    finally:
        del prog
        endOfRun(args)
    
if __name__ == '__main__':
    main()
    
