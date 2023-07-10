import copy
import json

DEFAULT_CONFIGFILE="""
{
    "version" : 3,
    "type" : "",
    "camera" : [
        {
            "serial" : "0",
            "type" : "",
            "trafo" : [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        }
    ]
}
"""

FILTER_PARAMS_REALSENSE=dict(
    do_threshold=True,
    threshold_near=0.2,
    threshold_far=4,

    depth_x_erosion=2,
    depth_y_erosion=2,
    
    do_decimation=True,
    decimation_value=2,
    
    do_spatial=True,
    spatial_iterations=1,
    spatial_alpha=0.5,
    spatial_delta=20,
    spatial_filling=1,
    
    do_temporal=True,
    temporal_alpha=0.4,
    temporal_delta=20,
    temporal_percistency=3,
)

SYSTEM_PARAMS_REALSENSE=dict(
    usb2width=640,
    usb2height=480,
    usb2fps=15,
    usb3width=1280,
    usb3height=720,
    usb3fps=30,
    usb2allowed=False,
    exposure=-1,
    whitebalance=-1,
    backlight_compensation=False,
    laser_power=360,
    density_preferred=True
)

SKELETON_PARAMS_REALSENSE=None

FILTER_PARAMS_KINECT=dict(
    do_threshold=True,
    threshold_near=0.2,
    threshold_far=4,

    depth_x_erosion=1,
    depth_y_erosion=1,
)

SYSTEM_PARAMS_KINECT=dict(
    color_height=720,
    depth_height=576,
    fps=15,
    sync_master_serial="",
    single_tile=-1,
    colormaster=True,
    color_exposure_time=-1,
    color_whitebalance=-1,
    color_backlight_compensation=-1,
    color_brightness=-1,
    color_contrast=-1,
    color_saturation=-1,
    color_sharpness=-1,
    color_gain=-1,
    color_powerline_frequency=-1,
    map_color_to_depth=False
)

SKELETON_PARAMS_KINECT=dict(
    sensor_orientation=-1,
    processing_mode=-1,
    model_path=""
)

DEFAULT_FILENAME="cameraconfig.json"
DEFAULT_TYPE="realsense"
DEFAULT_FILTER_PARAMS=FILTER_PARAMS_REALSENSE
DEFAULT_SYSTEM_PARAMS=SYSTEM_PARAMS_REALSENSE
DEFAULT_SKELETON_PARAMS=SKELETON_PARAMS_REALSENSE

def selectCameraType(cameraType):
    global DEFAULT_TYPE, DEFAULT_FILTER_PARAMS, DEFAULT_SYSTEM_PARAMS, DEFAULT_SKELETON_PARAMS
    DEFAULT_TYPE = cameraType
    DEFAULT_FILTER_PARAMS = globals()[f'FILTER_PARAMS_{cameraType.upper()}']
    DEFAULT_SYSTEM_PARAMS = globals()[f'SYSTEM_PARAMS_{cameraType.upper()}']
    DEFAULT_SKELETON_PARAMS = globals()[f'SKELETON_PARAMS_{cameraType.upper()}']

class CameraConfig:

    def __init__(self, confFilename, read=True):
        self.confFilename = confFilename
        self.serials = []
        self.matrices = []
        self.tree = None
        if read:
            self._readConf(self.confFilename)
            self._parseConf()
        
    def _readConf(self, confFilename):
        self.tree = json.load(open(confFilename))

    def loadConf(self, confString):
        self.tree = json.loads(confString)
        self._parseConf()
        
    def copyFrom(self, other):
        self.tree = copy.deepcopy(other.tree)
        self._parseConf()
    
    def _setcameratype(self, type):
        if 'type' in self.tree:
            assert self.tree['type'] in ('', type)
        self.tree['type'] = type
        
    def _parseConf(self):
        self.serials = []
        self.matrices = []
        for camElt in self.tree['camera']:
            self._setcameratype(camElt['type'])
            serial = camElt['serial']
            assert serial
            trafo = camElt['trafo']
            self.serials.append(serial)
            self.matrices.append(trafo)
        
    def save(self):
        json.dump(self.tree, open(self.confFilename, 'w'), indent=2)
        
    def getcount(self):
        return len(self.serials)
        
    def getserials(self):
        return self.serials
        
    def getmatrix(self, tilenum):
        return self.matrices[tilenum]
        
    def setmatrix(self, tilenum, matrix):
        self.matrices[tilenum] = copy.deepcopy(matrix)
        self.tree['camera'][tilenum]['trafo'] = self.matrices[tilenum]
   