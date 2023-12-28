import sys
import os
import argparse
import json
from typing import Optional
import cwipc
from cwipc.net.abstract import *
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

from ._scriptsupport import *


DEFAULT_FILENAME = "cameraconfig.json"

def main():
    parser = ArgumentParser(description="Register cwipc cameras (realsense, kinect, virtual) so they produce overlapping point clouds.")
    def twofloats(s):
        f1, f2 = s.split(',')
        return float(f1), float(f2)
    parser.add_argument("--fromxml", action="store_true", help="Convert cameraconfig.xml to cameraconfig.json and exit")
    
    parser.add_argument("--clean", action="store_true", help=f"Remove old {DEFAULT_FILENAME} and calibrate from scratch")
    parser.add_argument("--nograb", metavar="PLYFILE", action="store", help=f"Don't use grabber but use .ply file grabbed earlier, using {DEFAULT_FILENAME} from same directory.")
    args = parser.parse_args()
    beginOfRun(args)
    reg = Registrator(args)
    reg.run()

class CameraConfig:
    def __init__(self, filename):
        self.filename = filename
        self.cameraconfig = dict()
        oldconfig = self.filename + '~'
        if os.path.exists(oldconfig):
            os.unlink(oldconfig)

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
        
    def run(self) -> bool:
        self.args.nodecode = True
        self.capturerFactory, self.capturerName = cwipc_genericsource_factory(self.args)
        if self.args.fromxml:
            # Special case: load XML config file name create JSON config file
            self.json_from_xml()
            return True
        if not self.open_capturer():
            return False
        # Get initial cameraconfig
        assert self.capturer
        self.cameraconfig.load(self.capturer.get_config())
        print(f"xxxjack cameraconfig {self.cameraconfig.get()}")
        self.cameraconfig.save()
    
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
    
