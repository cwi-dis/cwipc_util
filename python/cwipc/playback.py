import cwipc
import time
import os

class _Filesource:
    def __init__(self, filenames, loop=False, fps=None):
        self.filenames = list(filenames)
        self.loop = loop
        self.delta_t = 0
        self.earliest_return = time.time()
        if fps:
            self.delta_t = 1/fps
        
    def free(self):
        self.filenames = []
        
    def eof(self):
        return not self.filenames
        
    def get(self):
        if not self.filenames:
            return None
        fn = self.filenames.pop(0)
        if self.loop: self.filenames.append(fn)
        rv = self._get(fn)
        if time.time() < self.earliest_return:
            time.sleep(self.earliest_return - time.time())
        self.earliest_return = time.time() + self.delta_t
        return rv
        
    def _get(self, fn):
        return cwipc.cwipc_read(fn, int(time.time()))        
        
class _DumpFilesource(_Filesource):
    def _get(self, fn):
        return cwipc.cwipc_read_debugdump(fn)        
    
def cwipc_playback(dir_or_files, ply=True, loop=False, fps=None):
    """Return cwipc_source-like object that reads .ply or .cwipcdump files from a directory or list of filenames"""
    if isinstance(dir_or_files, str):
        ext = ".ply" if ply else ".cwipcdump"
        filenames = filter(lambda fn : fn.lower().endswith(ext), os.listdir(dir_or_files))
        if not filenames:
            raise cwipc.CwipcError(f"No {ext} files in {dir_or_files}")
        filenames = sorted(filenames)
        filenames = list(filenames)
        filenames = map(lambda fn: os.path.join(dir_or_files, fn), filenames)
        filenames = list(filenames)
        dir_or_files = filenames
    if ply:
        return _Filesource(dir_or_files, loop, fps)
    else:
        return _DumpFilesource(dir_or_files, loop, fps)
        
