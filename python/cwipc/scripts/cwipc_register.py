import sys
import os
import time
import shutil
import argparse
import json
from typing import Optional, List, cast, Tuple, Dict
import numpy
import cwipc
from cwipc.net.abstract import *
from ._scriptsupport import *
from cwipc.registration.abstract import *
from cwipc.registration.util import *
import cwipc.registration.coarse
import cwipc.registration.fine
import cwipc.registration.analyze
import cwipc.registration.multicamera
from cwipc.io.visualizer import Visualizer

try:
    import cwipc.realsense2
except ModuleNotFoundError:
    cwipc.realsense2 = None
except FileNotFoundError:
    cwipc.realsense2 = None
try:
    import cwipc.kinect
except ModuleNotFoundError:
    cwipc.kinect = None
except FileNotFoundError:
    cwipc.kinect = None


DEFAULT_FILENAME = "cameraconfig.json"

class RegistrationVisualizer(Visualizer):
    reload_cameraconfig_callback : Optional[Callable[[], None]]

    def __init__(self, *args, **kwargs):
        Visualizer.__init__(self, *args, **kwargs)
        self.captured_pc : Optional[cwipc_wrapper] = None
        self.reload_cameraconfig_callback = None

    def write_current_pointcloud(self):
        self.captured_pc = self.cur_pc
        self.cur_pc = None
        self.stop()

    def reload_cameraconfig(self) -> None:
        super().reload_cameraconfig()
        if self.reload_cameraconfig_callback:
            self.reload_cameraconfig_callback()

def main():
    parser = ArgumentParser(description="Register cwipc cameras (realsense, kinect, virtual) so they produce overlapping point clouds.")
    def twofloats(s):
        f1, f2 = s.split(',')
        return float(f1), float(f2)
    parser.add_argument("--fromxml", action="store_true", help="Convert cameraconfig.xml to cameraconfig.json and exit")
    parser.add_argument("--guided", action="store_true", help="Guide me through the whole registration procedure")
    parser.add_argument("--tabletop", action="store_true", help="Do static registration of one camera, 1m away at 1m height")
    parser.add_argument("--noregister", action="store_true", help="Don't do any registration, only create cameraconfig.json if needed")
    parser.add_argument("--clean", action="store_true", help=f"Remove old {DEFAULT_FILENAME} and calibrate from scratch")

    parser.add_argument("--interactive", action="store_true", help="Interactive mode: show pointclouds (and optional RGB images). w command will attempt registration.")
    parser.add_argument("--rgb", action="store_true", help="Show RGB captures in addition to point clouds")
    parser.add_argument("--rgb_cw", action="store_true", help="When showing RGB captures first rotate the 90 degrees clockwise")
    parser.add_argument("--rgb_ccw", action="store_true", help="When showing RGB captures first rotate the 90 degrees counterclockwise")
    
    parser.add_argument("--coarse", action="store_true", help="Do coarse registration (default: only if needed)")
    parser.add_argument("--nofine", action="store_true", help="Don't do fine registration (default: always do it)")
    parser.add_argument("--analyze", action="store_true", help="Analyze the pointclouds and show a graph of the results")

    parser.add_argument("--algorithm_analyzer", action="store", help="Analyzer algorithm to use")
    parser.add_argument("--algorithm_multicamera", action="store", help="Fine alignment outer algorithm to use, for multiple cameras")
    parser.add_argument("--algorithm_fine", action="store", help="Fine alignment inner registration algorithm to use")
    parser.add_argument("--help_algorithms", action="store_true", help="Show available algorithms and a short description of them")

    parser.add_argument("--nograb", metavar="PLYFILE", action="store", help=f"Don't use grabber but use .ply file grabbed earlier, using {DEFAULT_FILENAME} from same directory.")
    parser.add_argument("--skip", metavar="N", type=int, action="store", help="Skip the first N captures")
    parser.add_argument("--no_aruco", action="store_true", help="Do coarse alignment with interactive selection (default: find aruco marker)")
    parser.add_argument("--conf_init", action="append", metavar="PATH=VALUE", help="If creating cameraconfig.json, set PATH to VALUE. Example: postprocessing.depthfilterparameters.threshold_far=3.0")
    
    parser.add_argument("--plot", action="store_true", help="After each fine aligner step show a graph of the results")
    parser.add_argument("--plotstyle", action="store", help="Plot style for the fine alignment step. Comma-separated list of 'count', 'cumulative', 'delta', 'all', 'log'.")
    parser.add_argument("--debug", action="store_true", help="Produce step-by-step pointclouds and cameraconfigs in directory cwipc_register_debug. Implies --verbose.")
    parser.add_argument("--dry_run", action="store_true", help="Don't modify cameraconfig file")
    
    parser.add_argument("recording", nargs='?', help="A directory with recordings (realsense or kinect) for which to do registration")
    args = parser.parse_args()
    beginOfRun(args)
    if args.debug:
        args.verbose = True
    if args.help_algorithms:
        print(cwipc.registration.analyze.HELP_ANALYZER_ALGORITHMS)
        print(cwipc.registration.fine.HELP_FINE_ALIGNMENT_ALGORITHMS)
        print(cwipc.registration.multicamera.HELP_MULTICAMERA_ALGORITHMS)
        return 0
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

    def get_serial_dict(self) -> Dict[int, str]:
        """Return a mapping from tilemask to camera serial number"""
        rv = {}
        if len(self.cameraconfig) == 1:
            mask = 0
        else:
            mask = 1
        for cam in self.cameraconfig["camera"]:
            serial = cam["serial"]
            rv[mask] = serial
            mask <<= 1
        return rv

    def load(self, jsondata : bytes) -> None:
        self.cameraconfig = json.loads(jsondata)
        self.init_transforms()
        self._dirty = False
        # Workaround for bug (only in realsense_playback?) 2023-12-30
        # Actually bug also exists, but differently, for kinect_offline.
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
        print(f"cwipc_register: saved {self.filename}")

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
    
    def set_entry_from_string(self, arg : str) -> bool:
        key, value = arg.split('=')
        value = eval(value)
        key_list = key.split(".")
        d = self.cameraconfig
        for k in key_list[:-1]:
            d = d[k]
        k = key_list[-1]
        if d.get(k) != value:
            d[k] = value
            self._dirty = 1
            return True
        return False


class Registrator:
    def __init__(self, args : argparse.Namespace):
        self.progname = sys.argv[0]
        self.capturerFactory : Optional[cwipc_tiledsource_factory_abstract] = None
        self.capturerName = None
        self.capturer = None
        self.args = args
        self.verbose = self.args.verbose
        self.show_plot = self.args.plot
        if args.plotstyle:
            self.show_plot = True
            cwipc.registration.analyze.set_default_plot_style(args.plotstyle)
        self.debug = self.args.debug
        self.dry_run = self.args.dry_run
        self.check_coarse_alignment = False # This can be a very expensive operation...
        #
        if self.args.guided:
            self.args.interactive = True
            self.args.rgb = True
        #
        # Coarse aligner class depends on what input we have. So there's no --algorithm option for this.
        #
        self.no_aruco = args.no_aruco
        if self.no_aruco:
            self.coarse_aligner_class = cwipc.registration.coarse.MultiCameraCoarseColorTarget
        elif self.args.rgb:
            self.coarse_aligner_class = cwipc.registration.coarse.MultiCameraCoarseArucoRgb
        else:
            self.coarse_aligner_class = cwipc.registration.coarse.MultiCameraCoarseAruco

        if self.args.algorithm_multicamera:
            self.multicamera_aligner_class = getattr(cwipc.registration.multicamera, args.algorithm_multicamera)
        else:
            self.multicamera_aligner_class = cwipc.registration.multicamera.DEFAULT_MULTICAMERA_ALGORITHM

        if self.args.algorithm_fine:
            self.alignment_class = getattr(cwipc.registration.fine, args.algorithm_fine)
        else:
            self.alignment_class = None # The fine alignment class is determined by the multicamera aligner chosen.

        if self.args.algorithm_analyzer:
            self.analyzer_class = getattr(cwipc.registration.analyze, args.algorithm_analyzer)
        else:
            self.analyzer_class = cwipc.registration.analyze.DEFAULT_ANALYZER_ALGORITHM

        if self.args.recording:
            if self.args.cameraconfig:
                print("cwipc_register: Cannot use --cameraconfig with a recording")
                sys.exit(1)
            self.args.cameraconfig = os.path.realpath(os.path.join(self.args.recording, DEFAULT_FILENAME))
        if not self.args.cameraconfig:
            self.args.cameraconfig = DEFAULT_FILENAME
        self.cameraconfig = CameraConfig(self.args.cameraconfig)
        if self.debug:
            if os.path.exists("cwipc_register_debug"):
                shutil.rmtree("cwipc_register_debug")
            os.mkdir("cwipc_register_debug")
            print(f"cwipc_register: Will produce debug files in ./cwipc_register_debug")

    def __del__(self):
        if self.capturer != None:
            self.capturer.free()
            self.capturer = None

    def prompt(self, message : str):
        print(f"{message}")
        
    def run(self) -> bool:
        if self.args.recording:
            # Special case: register a recording. First make sure we actually have a
            # cameraconfig file, otherwise generate it.
            if not self.initialize_recording():
                return False
            self.args.nodrop = True
        self.args.nodecode = True
        self.capturerFactory, self.capturerName = cwipc_genericsource_factory(self.args)
        if self.args.fromxml:
            # Special case: load XML config file name create JSON config file
            # Ensure that we have specified the capturer name, and that it is a supported one.
            supported = ["kinect", "k4aoffline", "realsense"]
            if not self.capturerName in supported:
                print("cwipc_register: Capturer needs to be specified for --fromxml")
                print(f"cwipc_register: Supported capturers: {' '.join(supported)}")
                return False
            self.json_from_xml()
            return True
        #
        # Now we really start
        #
        if not self.open_capturer():
            if self.args.recording:
                print(f"cwipc_register: Cannot open capturer, probably an issue with {self.args.cameraconfig}")
                return False
            print("cwipc_register: Cannot open capturer. Presume missing cameraconfig, try to create it.")
            self.create_cameraconfig()
            if not self.open_capturer():
                print("cwipc_register: Still cannot open capturer. Giving up.")
                return False
        assert self.capturer

        self._capture_some_frames(self.capturer)
                    
        self.cameraconfig.load(self.capturer.get_config())
        must_reload = False
        if not self.dry_run:
            anyChanged = False
            if self.args.conf_init:
                for setting in self.args.conf_init:
                    if self.cameraconfig.set_entry_from_string(setting):
                        anyChanged = True
            self.cameraconfig.save()
            if anyChanged:
                # Need to reload camera config if we made changes
                must_reload = True
        if self.args.noregister:
            return True
        if must_reload:
            self._reload_cameraconfig_to_capturer()
            must_reload = False
        if self.args.tabletop:
            assert self.cameraconfig.camera_count() == 1
            t = self.cameraconfig.get_transform(0)
            matrix = np.array(
                [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, -1.0, 0.0,  1.0],
                    [0.0, 0.0, -1.0,  0.60],
                    [0, 0, 0, 1]
                ])
            t.set_matrix(matrix)
            self.cameraconfig.save()
            return True
        if self.args.coarse and not self.cameraconfig.is_identity():
            print(f"cwipc_register: reset matrix")
            pass 
            for i in range(self.cameraconfig.camera_count()):
                t = self.cameraconfig.get_transform(i)
                t.set_matrix(transformation_identity())
            self.cameraconfig.save()
            self._reload_cameraconfig_to_capturer()
        if self.args.coarse or self.cameraconfig.is_identity():
            if self.args.guided:
                self.args.interactive = True
                self.args.rgb = True
                print(f"\n===================================================================", file=sys.stderr)
                print(f"===== Place Aruco marker at origin. ", file=sys.stderr)
                print(f"===== Examine RGB images and adjust cameras so marker is visible to all.", file=sys.stderr)
                print(f"===== Examine RGB images colors and exposure.", file=sys.stderr)
                print(f"===== Edit cameraconfig.json and change exposure, gain, backlight, etc. ", file=sys.stderr)
                print(f"===== Press c in point cloud window (or restart cwipc_register) to reload cameraconfig.json", file=sys.stderr)
                print(f"===== When all is good press w in point cloud window", file=sys.stderr)
                print(f"===================================================================\n", file=sys.stderr)
            new_pc = None
            while new_pc == None:
                self.prompt("Coarse registration: capturing aruco/color target")
                pc = self.capture()
                if self.debug:
                    self.save_pc(pc, "step1_capture_coarse")
                new_pc = self.coarse_registration(pc)
                pc.free()
                pc = None
            assert new_pc
            if self.debug:
                self.save_pc(new_pc, "step2_after_coarse")
            new_pc.free()
            new_pc = None
            if not self.dry_run:
                if self.verbose:
                    print(f"cwipc_register: save {self.cameraconfig.filename}")
                self.cameraconfig.save()
                must_reload = True
        else:
            if self.verbose:
                print(f"cwipc_register: skipping coarse registration, cameraconfig already has matrices")
        if self.cameraconfig.camera_count() > 1 and not self.args.nofine or self.args.analyze:
            while True:
                if must_reload:
                    self._reload_cameraconfig_to_capturer()
                    must_reload = False
                if self.args.guided:
                    self.args.rgb = False
                    print(f"\n===================================================================", file=sys.stderr)
                    print(f"===== Examine point cloud window.", file=sys.stderr)
                    print(f"===== Edit cameraconfig.json and change near, far, height_min, height_max, radius_filter. ", file=sys.stderr)
                    print(f"===== Press c in point cloud window (or restart cwipc_register) to reload cameraconfig.json", file=sys.stderr)
                    print(f"===== Ensure you get a clean capture including the floor.", file=sys.stderr)
                    print(f"===== Have a human (could be you) stand at the origin and ensure they are fully captured. ", file=sys.stderr)
                    print(f"===== You can use 1234 or 0 to see only per-camera point clouds. ", file=sys.stderr)
                    print(f"===== Press w in the point cloud window to capture.", file=sys.stderr)
                    print(f"===== Alternatively, press t (for timelapse) and ensure you yourself are in position in 5 seconds.", file=sys.stderr)
                    print(f"===== Or, if you are happy with the registration, press q")
                    print(f"===================================================================\n", file=sys.stderr)

                self.prompt("Fine registration: capturing human-sized object")
                pc = self.capture()
                if self.debug:
                    self.save_pc(pc, "step3_capture_fine")
                if self.args.analyze:
                    self.check_alignment(pc, 0, "analysis")
                else:
                    new_pc = self.fine_registration(pc)
                    pc.free()
                    pc = None
                    if self.debug:
                        self.save_pc(new_pc, "step4_after_fine")
                    new_pc.free()
                    new_pc = None
                    if not self.dry_run:
                        self.cameraconfig.save()
                if not self.args.guided:
                    break
                must_reload = True
        else:
            if self.verbose:
                print(f"cwipc_register: skipping fine registration, not needed")
        
        return False
    
    def _reload_cameraconfig_from_file(self) -> None:
        assert self.capturer
        print(f"cwipc_register: reload camerconfig from {self.cameraconfig.filename}")
        self.cameraconfig.load(self.capturer.get_config())

    def _reload_cameraconfig_to_capturer(self) -> None:
        assert self.capturer
        print(f"cwipc_register: reload {self.cameraconfig.filename} to capturer")
        self.capturer.reload_config(self.cameraconfig.filename)

    def initialize_recording(self) -> bool:
        if os.path.exists(self.args.cameraconfig):
            return True
        allfiles = []
        is_kinect = False
        is_realsense = False
        for fn in os.listdir(self.args.recording):
            if fn.startswith("."):
                continue
            if fn.lower().endswith(".mkv"):
                allfiles.append(fn)
                is_kinect = True
            if fn.lower().endswith(".bag"):
                allfiles.append(fn)
                is_realsense = True
        if is_realsense and is_kinect:
            print(f"cwipc_register: Directory {self.args.recording} contains both .mkv and .bag files")
            return False
        if not is_realsense and not is_kinect:
            print(f"cwipc_register: Directory {self.args.recording} contains neither .mkv nor .bag files")
            return False
        if is_realsense:
            camtype = "realsense_playback"
            # Create the camera definitions
            camera = [
                dict(filename=fn, type=camtype)
                    for fn in allfiles
            ]
            cameraconfig = dict(
                version=4,
                type=camtype,
                system=dict(),
                hardware=dict(),
                processing=dict(),
                filtering=dict(),
                camera=camera
            )
        elif is_kinect:
            camtype = "kinect_offline"
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
                skeleton=dict(),
                camera=camera
            )
        else:
            print(f"cwipc_register: Directory {self.args.recording} contains neither .mkv nor .bag files")
            return False

        json.dump(cameraconfig, open(self.args.cameraconfig, "w"), indent=4)
        if self.verbose:
            print(f"cwipc_register: Created {self.args.cameraconfig}")
        return True
    
    def json_from_xml(self):
        assert self.capturerFactory
        capturer = self.capturerFactory("cameraconfig.xml") # type: ignore
        self._capture_some_frames(capturer)
        json_data = capturer.get_config()
        open(self.args.cameraconfig, 'wb').write(json_data)

    def open_capturer(self) -> bool:
        assert self.capturerFactory
        # xxxjack this will eventually fail for generic capturer
        if not self.capturerName:
            print(f"cwipc_register: selected capturer does not need registration")
            return False
        # Step one: Try to open with an existing cameraconfig.
        if self.capturer != None:
            self.capturer.free()
            self.capturer = None
        try:
            self.capturer = self.capturerFactory()
        except cwipc.CwipcError:
            return False
        self.capturer.request_auxiliary_data("rgb")
        self.capturer.request_auxiliary_data("depth")
        self.capturer.request_auxiliary_data("timestamps")
        return True
    
    def create_cameraconfig(self) -> None:
        """Attempt to open a capturer without cameraconfig file. Then save the empty cameraconfig."""
        tmpFactory, _ = cwipc_genericsource_factory(self.args, autoConfig=True)
        tmpCapturer = tmpFactory()
        new_config = tmpCapturer.get_config()
        self.cameraconfig.load(new_config)
        tmpCapturer.free()
        tmpCapturer = None
        if not self.dry_run:
            self.cameraconfig.save()
            if self.verbose:
                print(f"cwipc_register: Saved {self.args.cameraconfig}")
        else:
            print(f"cwipc_register: Not saving {self.args.cameraconfig}, --dry-run specified.")
        
    def capture(self) -> cwipc.cwipc_wrapper:
        if self.args.nograb:
            pc = cwipc.cwipc_read(self.args.nograb, 0)
            return pc
        assert self.capturer
        if self.args.skip:
            if self.verbose:
                print(f"cwipc_register: skipping {self.args.skip} captures")
            for i in range(self.args.skip):
                ok = self.capturer.available(True)
                if ok:
                    pc = self.capturer.get()
                    if pc != None:
                        pc.free()
        if self.args.interactive:
            return self.interactive_capture()
        ok = self.capturer.available(True)
        assert ok
        pc = self.capturer.get()
        assert pc
        assert pc.count() > 0
        return cast(cwipc.cwipc_wrapper, pc)
    
    def interactive_capture(self) -> cwipc.cwipc_wrapper:
        visualizer = RegistrationVisualizer(self.args.verbose, nodrop=True, args=self.args, title="cwipc_register")
        visualizer.reload_cameraconfig_callback = self._reload_cameraconfig_from_file
        sourceServer = SourceServer(cast(cwipc_source_abstract, self.capturer), visualizer, self.args)
        sourceThread = threading.Thread(target=sourceServer.run, args=(), name="cwipc_view.SourceServer")
        visualizer.set_producer(sourceThread)
        visualizer.set_source(cast(cwipc_source_abstract, self.capturer))
        sourceThread.start()
        visualizer.run()
        captured_pc = visualizer.captured_pc
        sourceServer.stop()
        sourceServer.grabber = None # type: ignore
        sourceThread.join()
        del visualizer
        del sourceServer
        del sourceThread
        if not captured_pc:
            print("cwipc_register: no capture selected in interactive mode. Exiting.")
            sys.exit(1)
        return captured_pc

    def save_pc(self, pc : cwipc_wrapper, label : str) -> None:
        if self.debug:
            filename = f"cwipc_register_debug/{label}-pointcloud.ply"
            cwipc.cwipc_write(filename, pc)
            filename = f"cwipc_register_debug/{label}-cameraconfig.json"
            self.cameraconfig.save_to(filename)
            print(f"cwipc_register: Saved pointcloud and cameraconfig for {label}")
            
    def coarse_registration(self, pc : cwipc_wrapper) -> Optional[cwipc_wrapper]:
        if True or self.verbose:
            print(f"cwipc_register: Use coarse alignment class {self.coarse_aligner_class.__name__}")
        assert self.capturer
        aligner = self.coarse_aligner_class()
        aligner.verbose = self.verbose
        aligner.debug = self.debug
        aligner.add_tiled_pointcloud(pc)
        serial_dict = self.cameraconfig.get_serial_dict()
        aligner.set_serial_dict(serial_dict)
        aligner.set_grabber(self.capturer)
        start_time = time.time()
        ok = aligner.run()
        stop_time = time.time()
        if self.verbose:
            print(f"cwipc_register: coarse aligner ran for {stop_time-start_time:.3f} seconds")
        if not ok:
            print("cwipc_register: Could not do coarse registration")
            return None
        # Get the resulting transformations, and store them in cameraconfig.
        transformations = aligner.get_result_transformations()
        for cam_num in range(len(transformations)):
            matrix = transformations[cam_num]
            t = self.cameraconfig.get_transform(cam_num)
            t.set_matrix(matrix)
        # Get the newly aligned pointcloud to test for alignment, and return it
        new_pc = aligner.get_result_pointcloud_full()
        if self.check_coarse_alignment:
            correspondence = self.check_alignment(new_pc, 0, "after coarse registration")
            self.cameraconfig["correspondence"] = correspondence
        return new_pc

    def fine_registration(self, pc : cwipc_wrapper) -> cwipc_wrapper:
        if not self.verbose:
            # We only do this if we are not running verbosely (because then the alignment classes will print this info)
            self.check_alignment(pc, 0, "before fine registration")
        if True or self.verbose:
            print(f"cwipc_register: Use fine aligner class {self.multicamera_aligner_class.__name__}")
        multicam = self.multicamera_aligner_class()
        multicam.verbose = self.verbose
        if self.alignment_class:
            multicam.set_aligner_class(self.alignment_class)
        multicam.set_analyzer_class(self.analyzer_class)
        multicam.debug = self.debug
        # This number sets a threashold for the best possible alignment.
        # xxxjack it should be computed from the source point clouds
        original_capture_precision = 0.001

        multicam.show_plot = self.show_plot
        multicam.add_tiled_pointcloud(pc)
        for cam_index in range(self.cameraconfig.camera_count()):
            multicam.set_original_transform(cam_index, self.cameraconfig.get_transform(cam_index).get_matrix())
        start_time = time.time()
        ok = multicam.run()
        stop_time = time.time()
        if self.verbose:
            print(f"cwipc_register: multicamera fine aligner ran for {stop_time-start_time:.3f} seconds")
        if not ok:
            print("cwipc_register: Could not do multicamera fine registration")
            sys.exit(1)
        # Get the resulting transformations, and store them in cameraconfig.
        transformations = multicam.get_result_transformations()
        for cam_num in range(len(transformations)):
            matrix = transformations[cam_num]
            t = self.cameraconfig.get_transform(cam_num)
            t.set_matrix(matrix)
        # Get the newly aligned pointcloud to test for alignment, and return it
        new_pc = multicam.get_result_pointcloud_full()
        correspondence = self.check_alignment(new_pc, 0, "after fine registration")
        self.cameraconfig["correspondence"] = correspondence
        return new_pc

    def check_alignment(self, pc : cwipc_wrapper, original_capture_precision : float, label : str) -> float:
        assert self.analyzer_class
        if True or self.verbose:
            print(f"cwipc_register: Use analyzer class {self.analyzer_class.__name__}")
        analyzer = self.analyzer_class()
        analyzer.verbose = self.verbose
        analyzer.debug = self.debug
        analyzer.add_tiled_pointcloud(pc)
        analyzer.plot_label = label
        start_time = time.time()
        analyzer.run_twice()
        stop_time = time.time()
        print(f"cwipc_register: analyzer ran for {stop_time-start_time:.3f} seconds")
        results = analyzer.get_results()
        results.sort_by_weight(weightstyle='order')
        results.print_correspondences(label=f"cwipc_register: Sorted correspondences {label}")
        if self.show_plot:
            analyzer.plot(show=True)
        
        return results.overallCorrespondence      

    def _capture_some_frames(self, capturer : cwipc_tiledsource_abstract) -> None:
        # Capture some frames (so we know get_config() will have obtained all parameters).
        # Get initial cameraconfig and save it
        gotframes = 0
        while gotframes < 3:
            ok = capturer.available(True)
            if ok:
                pc = capturer.get()
                if pc != None:
                    if self.verbose:
                        print(f"cwipc_register: dropped pc with {pc.count()} points")
                    pc.free()
                    pc= None

                    gotframes += 1
        if self.verbose:
            print(f"cwipc_register: captured {gotframes} frames to ensure config stability")

if __name__ == '__main__':
    main()
    
