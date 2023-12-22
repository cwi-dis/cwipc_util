import sys
import argparse
import json
import cwipc
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

from .calibrator import Calibrator
from .filegrabber import FileGrabber
from .livegrabber import LiveGrabber
from . import cameraconfig
from .cameraconfig import DEFAULT_FILENAME
from .targets import targets
from .._scriptsupport import *

def main():
    print(f"WARNING: {sys.argv[0]} is deprecated. Use cwipc_register.")
    
    parser = ArgumentParser(description="Calibrate cwipc_realsense2 or cwipc_kinect capturer")
    def twofloats(s):
        f1, f2 = s.split(',')
        return float(f1), float(f2)
    operation = parser.add_argument_group("Operation").add_mutually_exclusive_group(required=True)
    operation.add_argument("--auto", action="store_true", help=f"Auto-create cameraconfig.json, if it doesn't exist, with default position and orientation")
    operation.add_argument("--coarse", action="store_true", help="Do coarse (manual) calibration step")
    operation.add_argument("--fine", action="store_true", help="Do fine calibration step")
    operation.add_argument("--fromxml", action="store_true", help="Convert cameraconfig.xml to cameraconfig.json and exit")
    operation.add_argument("--list", action="store_true", help="List available targets for coarse calibration")
    
    parser.add_argument("--clean", action="store_true", help=f"Remove old {DEFAULT_FILENAME} and calibrate from scratch")
    parser.add_argument("--reuse", action="store_true", help=f"Reuse existing {DEFAULT_FILENAME}")
    parser.add_argument("--nograb", metavar="PLYFILE", action="store", help=f"Don't use grabber but use .ply file grabbed earlier, using {DEFAULT_FILENAME} from same directory.")
    parser.add_argument("--target", action="store", metavar="NAME", default="a4floor", help="Specify which target to use for coarse calibration. --list to see options.")
    parser.add_argument("--corr", action="store", type=float, metavar="D", help="Set fine calibration max corresponding point distance (Default=0.01)", default=0.01)
    parser.add_argument("--finspect", action="store_true", help="Visually inspect result of each fine calibration step")
    args = parser.parse_args()
    beginOfRun(args)
    bbox = None
    if args.list:
        for name, target in targets.items():
            print(f'{name}\n\t{target["description"]}')
        sys.exit(0)
    capturerFactory, capturerName = cwipc_genericsource_factory(args, autoConfig=True)
    if args.fromxml:
        # Special case: load XML config file name create JSON config file
        capturer = capturerFactory("cameraconfig.xml") # type: ignore
        json_data = capturer.get_config()
        open('cameraconfig.json', 'wb').write(json_data)
        return 0
    # xxxjack this will eventually fail for generic capturer
    if not capturerName:
        print(f"{sys.argv[0]}: selected capturer does not need calibration")
        sys.exit(1)
    if capturerName == "auto":
        print(f"{sys.argv[0]}: please specify --kinect or --realsense")
        sys.exit(1)
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
    
