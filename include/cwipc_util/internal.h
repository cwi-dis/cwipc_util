#ifndef _cwipc_util_internal_h_
#define _cwipc_util_internal_h_

struct CwipcBaseCameraConfig {
	std::string type;
};

struct CwipcBaseCaptureConfig {
	std::string type;
};

class CwipcBaseCamera {
protected:
	std::string type;
};

class CwipcBaseCapture {
protected:
	std::string type;
};

#endif // _cwipc_util_internal_h_
