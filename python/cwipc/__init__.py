from .util import *

def cwipc_opengl_window(*args, **kwargs):
    from . import opengl
    return opengl.cwipc_opengl_window(*args, **kwargs)
