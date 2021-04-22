import sys
import argparse

try:
    import cwipc.realsense2
    # Workaround for https://github.com/intel-isl/Open3D/issues/3283
    _ = cwipc.realsense2._cwipc_realsense2_dll()
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
    parser = ArgumentParser(description="Calibrate cwipc_realsense2 or cwipc_kinect capturer")
    def twofloats(s):
        f1, f2 = s.split(',')
        return float(f1), float(f2)
    parser.add_argument("--auto", action="store_true", help=f"Attempt to auto-install {DEFAULT_FILENAME}, if needed")
    parser.add_argument("--clean", action="store_true", help=f"Remove old {DEFAULT_FILENAME} and calibrate from scratch")
    parser.add_argument("--reuse", action="store_true", help=f"Reuse existing {DEFAULT_FILENAME}")
    parser.add_argument("--nograb", metavar="PLYFILE", action="store", help=f"Don't use grabber but use .ply file grabbed earlier, using {DEFAULT_FILENAME} from same directory.")
    parser.add_argument("--noinspect", action="store_true", help="Don't inspect pointclouds after grabbing")
    parser.add_argument("--target", action="store", metavar="NAME", default="a4", help="Specify which target to use for coarse calibration. --list to see options.")
    parser.add_argument("--list", action="store_true", help="List available targets")
    parser.add_argument("--nocoarse", action="store_true", help="Skip coarse (manual) calibration step")
    parser.add_argument("--nofine", action="store_true", help="Skip fine (automatic) calibration step")
    parser.add_argument("--bbox", action="store", type=float, nargs=6, metavar="N", help="Set bounding box (in meters, xmin xmax etc) before fine calibration")
    parser.add_argument("--corr", action="store", type=float, metavar="D", help="Set fine calibration max corresponding point distance (Default=0.01)", default=0.01)
    parser.add_argument("--finspect", action="store_true", help="Visually inspect result of each fine calibration step")
    parser.add_argument("--depth", type=twofloats, action="store", metavar="MIN,MAX", help="Near and far distance in meters between camera(s) and subject")
    parser.add_argument("--height", type=twofloats, action="store", metavar="MIN,MAX", help="Min and max Y value in meters, sets height filter for pointclouds")
    args = parser.parse_args()
    bbox = None
    if args.list:
        for name, target in targets.items():
            print(f'{name}\n\t{target["description"]}')
        sys.exit(0)
    capturerFactory, capturerName = cwipc_genericsource_factory(args)
    if not capturerName:
        print(f"{sys.argv[0]}: selected capturer does not need calibration")
        sys.exit(1)
    cameraconfig.selectCameraType(capturerName)
    if args.bbox:
        bbox = args.bbox
        assert len(bbox) == 6
        assert type(1.0*bbox[0]*bbox[1]*bbox[2]*bbox[3]*bbox[4]*bbox[5]) == float
    refpoints = targets[args.target]["points"]
    prog = Calibrator(refpoints)
    if args.height:
        prog.setheight(*args.height)
    if args.depth:
        prog.setdepth(*args.depth)
    if args.nograb:
        grabber = FileGrabber(args.nograb)
    else:
        grabber = LiveGrabber(capturerFactory)
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
        else:
            noinspect = args.noinspect
            if args.nograb and args.nocoarse:
                noinspect = True
            prog.grab(noinspect)
        
            if args.nocoarse: 
                prog.skip_coarse()
            else:
                prog.run_coarse()
            
            if bbox:
                prog.apply_bbox(bbox)
            
            if args.nofine: 
                prog.skip_fine()
            else:
                prog.run_fine(args.corr, args.finspect)
            
        prog.save()
    finally:
        del prog
    
if __name__ == '__main__':
    main()
    
