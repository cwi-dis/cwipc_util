import sys
from typing import Optional
import argparse
import traceback
import cwipc
from cwipc.registration.fine import RegistrationComputer_ICP_Point2Point

class TransformFinder:
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.source_pc : Optional[cwipc.cwipc_wrapper] = None
        self.target_pc : Optional[cwipc.cwipc_wrapper] = None
        self.aligner = RegistrationComputer_ICP_Point2Point()
        self.aligner.verbose = verbose

    def load_source(self, source: str):
        self.source_pc = cwipc.cwipc_read(source, 0)

    def load_target(self, target: str):
        self.target_pc = cwipc.cwipc_read(target, 0)

    def run(self):
        assert self.source_pc
        assert self.target_pc
        self.aligner.set_reference_pointcloud(self.target_pc)
        self.aligner.add_tiled_pointcloud(self.source_pc)
        self.aligner.run(0)
        transform = self.aligner.get_result_transformation()
        print("Transform matrix:")
        print(transform)

def main():
    parser = argparse.ArgumentParser(description="Find transform between two pointclouds", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("target", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--debugpy", action="store_true", help="Wait for debugpy client to attach")
    args = parser.parse_args()
    if args.debugpy:
        import debugpy
        debugpy.listen(5678)
        print(f"{sys.argv[0]}: waiting for debugpy attach on 5678", flush=True)
        debugpy.wait_for_client()
        print(f"{sys.argv[0]}: debugger attached")        
    finder = TransformFinder(verbose=args.verbose)
    finder.load_source(args.source)
    finder.load_target(args.target)
    finder.run()
    
if __name__ == '__main__':
    main()
    
    
    
