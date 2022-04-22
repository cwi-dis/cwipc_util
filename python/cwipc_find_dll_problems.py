import dlltracer
import sys

print("----------- Loading cwipc_util -----------------")
with dlltracer.Trace(out=sys.stdout):
    import cwipc.util
    g = cwipc.util.cwipc_synthetic()
    g.free()
print("----------- Loading cwipc_codec -----------------")
with dlltracer.Trace(out=sys.stdout):
    import cwipc.codec
    g = cwipc.codec.cwipc_new_encoder()
    g.free()
print("----------- Loading cwipc_realsense2 -----------------")
with dlltracer.Trace(out=sys.stdout):
    import cwipc.realsense2
    g = cwipc.realsense2.cwipc_realsense2()
    g.free()
print("----------- Loading cwipc_kinect -----------------")
with dlltracer.Trace(out=sys.stdout):
    import cwipc.kinect
    g = cwipc.kinect.cwipc_kinect()
    g.free()
