import sys
import argparse

from .calibrator import Calibrator
from .filegrabber import FileGrabber
from .livegrabber import LiveGrabber
from .cameraconfig import DEFAULT_FILENAME
#
# Version 2 calibration cross: the one made with little balls and the forward spar
#
POINTS_V2 = [
    (0.25, 1, 0, 255, 0, 0),        # Red ball at Right of the cross
    (0, 1.25, 0, 0, 0, 255),        # Blue ball at top of the cross (because the sky is blue)
    (0, 1.1, -0.25, 255, 127, 0),   # Orange ball pointing towards the viewer (-Z) because everyone likes orange
    (-0.25, 1, 0, 255, 255, 0),     # Yellow ball at left of cross because it had to go somewhere
    (0, 0, 0, 0, 0, 0),             # Black point at 0,0,0
    (-1, 0, -1, 0, 0, 0),             # Black point at -1,0,-1
    (-1, 0, 1, 0, 0, 0),             # Black point at -1,0,1
    (1, 0, -1, 0, 0, 0),             # Black point at 1,0,-1
    (1, 0, 1, 0, 0, 0),             # Black point at 1,0,1
    (0, 1, 0, 0, 0, 0),             # Black point at cross
    (0, 1.1, 0, 0, 0, 0),           # Black point at forward spar
]
#
# Version 3 calibration cross: the one with the leds and the uneven arms
#
POINTS_V3 = [
    (-0.26, 1.04, 0, 255, 0, 0),        # Red LED at left of the cross
    (0, 1.44, 0, 0, 0, 255),        # Blue LED at top of the cross (because the sky is blue)
    (0, 1.31, -0.26, 0, 255, 0),   # Green LED pointing towards the viewer (-Z)
    (0.26, 1.17, 0, 255, 255, 0),     # Yellow LED at right of cross because it had to go somewhere
    (0, 0, 0, 0, 0, 0),             # Black point at 0,0,0
    (-1, 0, -1, 0, 0, 0),             # Black point at -1,0,-1
    (-1, 0, 1, 0, 0, 0),             # Black point at -1,0,1
    (1, 0, -1, 0, 0, 0),             # Black point at 1,0,-1
    (1, 0, 1, 0, 0, 0),             # Black point at 1,0,1
]
#
# VideoLat calibration device, for frontal camera in desktop position
POINTS_VIDEOLAT = [
    (-0.040, 1.275,  0.000, 127, 127, 127),    # topleft back row
    ( 0.040, 1.275,  0.000, 127, 127, 127),    # topright back row
    (-0.015, 1.250, -0.025, 127, 127, 127),    # topleft front row
    ( 0.015, 1.250, -0.025, 127, 127, 127),    # topright front row
    (-0.015, 1.225, -0.025, 127, 127, 127),    # bottomleft front row
    ( 0.015, 1.225, -0.025, 127, 127, 127),    # bottomright front row
    (-0.040, 1.200,  0.000, 127, 127, 127),    # bottomleft back row
    ( 0.040, 1.200,  0.000, 127, 127, 127),    # bottomright back row
]
#
# Rubiks cube
RH = 0.028 # Half the width/height/depth of the cube
POINTS_RUBIK = [
    (-RH, 1+RH, 0, 127, 127, 127),  # topleft
    (+RH, 1+RH, 0, 127, 127, 127),  # topright
    (-RH, 1-RH, 0, 127, 127, 127),  # botleft
    (+RH, 1-RH, 0, 127, 127, 127),  # botright
]
      
def main():
    parser = argparse.ArgumentParser(description="Calibrate cwipc_realsense2 capturer")
    def twofloats(s):
        f1, f2 = s.split(',')
        return float(f1), float(f2)
    parser.add_argument("--auto", action="store_true", help=f"Attempt to auto-install {DEFAULT_FILENAME}, if needed")
    parser.add_argument("--clean", action="store_true", help=f"Remove old {DEFAULT_FILENAME} and calibrate from scratch")
    parser.add_argument("--reuse", action="store_true", help=f"Reuse existing {DEFAULT_FILENAME}")
    parser.add_argument("--nograb", metavar="PLYFILE", action="store", help=f"Don't use grabber but use .ply file grabbed earlier, using {DEFAULT_FILENAME} from same directory.")
    parser.add_argument("--noinspect", action="store_true", help="Don't inspect pointclouds after grabbing")
    parser.add_argument("--nocoarse", action="store_true", help="Skip coarse (manual) calibration step")
    parser.add_argument("--nofine", action="store_true", help="Skip fine (automatic) calibration step")
    parser.add_argument("--crossv3", action="store_true", help="For coarse calibration, use version 3 calibration cross (with the LEDs) in stead of the v2 rubber ball cross")
    parser.add_argument("--videolat", action="store_true", help="For coarse calibration, use videolat flattened pyramid in stead of the v2 rubber ball cross")
    parser.add_argument("--rubik", action="store_true", help="For coarse calibration, use Rubiks cube, points on forward-looking face")
    parser.add_argument("--bbox", action="store", type=float, nargs=6, metavar="N", help="Set bounding box (in meters, xmin xmax etc) before fine calibration")
    parser.add_argument("--corr", action="store", type=float, metavar="D", help="Set fine calibration max corresponding point distance", default=0.01)
    parser.add_argument("--finspect", action="store_true", help="Visually inspect result of each fine calibration step")
    parser.add_argument("--depth", type=twofloats, action="store", metavar="MIN,MAX", help="Near and far distance in meters between camera(s) and subject")
    parser.add_argument("--height", type=twofloats, action="store", metavar="MIN,MAX", help="Min and max Y value in meters, sets height filter for pointclouds")
    args = parser.parse_args()
    bbox = None
    if args.bbox:
        bbox = args.bbox
        assert len(bbox) == 6
        assert type(1.0*bbox[0]*bbox[1]*bbox[2]*bbox[3]*bbox[4]*bbox[5]) == float
    refpoints = POINTS_V2
    if args.crossv3:
        refpoints = POINTS_V3
    if args.videolat:
        refpoints = POINTS_VIDEOLAT
    if args.rubik:
        refpoints = POINTS_RUBIK
    prog = Calibrator(refpoints)
    if args.height:
        prog.setheight(*args.height)
    if args.depth:
        prog.setdepth(*args.depth)
    if args.nograb:
        grabber = FileGrabber(args.nograb)
    else:
        grabber = LiveGrabber()
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
    
