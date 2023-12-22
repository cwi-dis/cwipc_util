import sys
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

class Registrator:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.verbose = self.args.verbose
        self.cameraconfig = self.args.cameraconfig
        if not self.cameraconfig:
            self.cameraconfig = DEFAULT_FILENAME
        self.progname = sys.argv[0]
        self.capturerFactory : Optional[cwipc_source_factory_abstract] = None
        self.capturerName = None
        self.capturer = None
        
    def run(self) -> bool:
        self.capturerFactory, self.capturerName = cwipc_genericsource_factory(self.args, autoConfig=True)
        if self.args.fromxml:
            # Special case: load XML config file name create JSON config file
            self.json_from_xml()
            return True
        if not self.open_capturer():
            return False
    
        return False
    
    def json_from_xml(self):
        capturer = capturerFactory("cameraconfig.xml") # type: ignore
        json_data = capturer.get_config()
        open(self.cameraconfig, 'wb').write(json_data)

    def open_capturer(self) -> bool:
        assert self.capturerFactory
        # xxxjack this will eventually fail for generic capturer
        if not self.capturerName:
            print(f"{self.progname}: selected capturer does not need calibration")
            return False
        if self.capturerName == "auto":
            print(f"{self.progname}: please specify --kinect or --realsense")
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
    
