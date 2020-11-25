import copy
import xml.etree.ElementTree as ET

CONFIGFILE_REALSENSE="""<?xml version="1.0" ?>
<file>
    <CameraConfig>
        <system usb2width="640" usb2height="480" usb2fps="15" usb3width="1280" usb3height="720" usb3fps="30" usb2allowed="0" />
        <postprocessing density="1" height_min="0" height_max="0" depthfiltering="1" greenscreenremoval="0" cloudresolution="0">
            <depthfilterparameters  />
        </postprocessing>
        <camera serial="0" type="">
            <trafo>
                <values v00="1" v01="0" v02="0" v03="0" v10="0" v11="1" v12="0" v13="0" v20="0" v21="0" v22="1" v23="0" v30="0" v31="0" v32="0" v33="1"  />
            </trafo>
        </camera>
    </CameraConfig>
</file>
"""

CONFIGFILE_KINECT="""<?xml version="1.0" ?>
<file>
    <CameraConfig>
        <system color_height="720" depth_height="576" fps="15" sync_master_serial="" colormaster="1" color_exposure_time="-1" color_whitebalance="-1" color_backlight_compensation="-1" color_brightness="-1" color_contrast="-1" color_saturation="-1" color_sharpness="-1" color_gain="-1" color_powerline_frequency="-1"/>
        <postprocessing density="1" height_min="0" height_max="0" depthfiltering="1" greenscreenremoval="0" cloudresolution="0">
            <depthfilterparameters  />
        </postprocessing>
        <camera serial="0" type="">
            <trafo>
                <values v00="1" v01="0" v02="0" v03="0" v10="0" v11="1" v12="0" v13="0" v20="0" v21="0" v22="1" v23="0" v30="0" v31="0" v32="0" v33="1"  />
            </trafo>
        </camera>
    </CameraConfig>
</file>
"""

FILTER_PARAMS_REALSENSE=dict(
    threshold_near="0.2",
    threshold_far="4",
    do_decimation="0",
    decimation_value="1",
    spatial_iterations="1",
    spatial_alpha="0.5",
    spatial_delta="20",
    spatial_filling="1",
    do_temporal="0",
    temporal_alpha="0.4",
    temporal_delta="20",
    temporal_percistency="3",
)
FILTER_PARAMS_KINECT=dict(
    threshold_near="0.2",
    threshold_far="4",
)

DEFAULT_FILENAME="cameraconfig.xml"
DEFAULT_TYPE="realsense"
DEFAULT_FILTER_PARAMS=FILTER_PARAMS_REALSENSE
DEFAULT_CONFIGFILE=CONFIGFILE_REALSENSE

def selectCameraType(cameraType):
    global DEFAULT_TYPE, DEFAULT_FILTER_PARAMS, DEFAULT_CONFIGFILE
    DEFAULT_TYPE = cameraType
    DEFAULT_FILTER_PARAMS = globals()[f'FILTER_PARAMS_{cameraType.upper()}']
    DEFAULT_CONFIGFILE = globals()[f'CONFIGFILE_{cameraType.upper()}']
    
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
        self.tree = ET.parse(confFilename)
        
    def copyFrom(self, other):
        self.tree = copy.deepcopy(other.tree)
        self._parseConf()
        
    def fillDefault(self):
        root = ET.fromstring(DEFAULT_CONFIGFILE)
        paramElt = root.find('CameraConfig/postprocessing/depthfilterparameters')
        for k, v in DEFAULT_FILTER_PARAMS.items():
            paramElt.set(k, v)
        self.tree = ET.ElementTree(root)
        
        self._parseConf()
        
    def _parseConf(self):
        root = self.tree.getroot()
        for camElt in root.findall('CameraConfig/camera'):
            serial = camElt.attrib['serial']
            assert serial
            trafoElt = camElt.find('trafo')
            valuesElt = trafoElt.find('values')
            va = valuesElt.attrib
            trafo = [
                [float(va['v00']), float(va['v01']), float(va['v02']), float(va['v03'])],
                [float(va['v10']), float(va['v11']), float(va['v12']), float(va['v13'])],
                [float(va['v20']), float(va['v21']), float(va['v22']), float(va['v23'])],
                [float(va['v30']), float(va['v31']), float(va['v32']), float(va['v33'])],
            ]
            self.serials.append(serial)
            self.matrices.append(trafo)
        
    def save(self):
        self.tree.write(self.confFilename)
        
    def getcount(self):
        return len(self.serials)
        
    def getserials(self):
        return self.serials
        
    def getmatrix(self, tilenum):
        return self.matrices[tilenum]
        
    def addcamera(self, serial):
        root = self.tree.getroot()
        firstCamElt = root.find('CameraConfig/camera')
        newCamElt = copy.deepcopy(firstCamElt)
        newCamElt.set('serial', serial)
        newCamElt.set('type', DEFAULT_TYPE)
        print(f'xxxjack set type to {DEFAULT_TYPE}')
        ccElt = root.find('CameraConfig')
        ccElt.append(newCamElt)
        
        self.serials.append(serial)
        matrix = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        self.matrices.append(matrix)
        
        self.setmatrix(len(self.serials)-1, matrix)
        
    def setmatrix(self, tilenum, matrix):
        self.matrices[tilenum] = copy.deepcopy(matrix)
        serial = self.serials[tilenum]
        root = self.tree.getroot()
        camElt = root.find(f"CameraConfig/camera[@serial='{serial}']")
        trafoElt = camElt.find('trafo')
        valuesElt = trafoElt.find('values')
        valuesElt.set('v00', str(matrix[0][0]))
        valuesElt.set('v01', str(matrix[0][1]))
        valuesElt.set('v02', str(matrix[0][2]))
        valuesElt.set('v03', str(matrix[0][3]))
        valuesElt.set('v10', str(matrix[1][0]))
        valuesElt.set('v11', str(matrix[1][1]))
        valuesElt.set('v12', str(matrix[1][2]))
        valuesElt.set('v13', str(matrix[1][3]))
        valuesElt.set('v20', str(matrix[2][0]))
        valuesElt.set('v21', str(matrix[2][1]))
        valuesElt.set('v22', str(matrix[2][2]))
        valuesElt.set('v23', str(matrix[2][3]))
        valuesElt.set('v30', str(matrix[3][0]))
        valuesElt.set('v31', str(matrix[3][1]))
        valuesElt.set('v32', str(matrix[3][2]))
        valuesElt.set('v33', str(matrix[3][3]))
        
    def setserial(self, tilenum, serial):
        oldSerial = self.serials[tilenum]
        self.serials[tilenum] = serial
        root = self.tree.getroot()
        camElt = root.find(f"CameraConfig/camera[@serial='{oldSerial}']")
        camElt.set('serial', serial)
        camElt.set('type', DEFAULT_TYPE)
        
    def setdistance(self, threshold_near, threshold_far):
        root = self.tree.getroot()
        ppElt = root.find('CameraConfig/postprocessing')
        dfElt = ppElt.find('depthfilterparameters')
        dfElt.set('threshold_near', str(threshold_near))
        dfElt.set('threshold_far', str(threshold_far))
        
    def setheight(self, height_min, height_max):
        root = self.tree.getroot()
        ppElt = root.find('CameraConfig/postprocessing')
        ppElt.set('height_min', str(height_min))
        ppElt.set('height_max', str(height_max))
