import sys
from typing import Optional
import argparse
import traceback
import cwipc
from cwipc.registration.fine import RegistrationComputer_ICP_Point2Point
from cwipc.registration.analyze import RegistrationAnalyzer
from cwipc.registration.util import transformation_topython

class TransformFinder:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.analyze = args.analyze
        self.dump = args.dump
        self.verbose = args.verbose
        self.source_pc : Optional[cwipc.cwipc_wrapper] = None
        self.target_pc : Optional[cwipc.cwipc_wrapper] = None
        self.result_pc : Optional[cwipc.cwipc_wrapper] = None
        self.aligner = RegistrationComputer_ICP_Point2Point()
        self.aligner.verbose = self.verbose
            

    def load_source(self, source: str):
        self.source_pc = cwipc.cwipc_read(source, 0)

    def load_target(self, target: str):
        self.target_pc = cwipc.cwipc_read(target, 0)

    def run(self):
        assert self.source_pc
        assert self.target_pc
        if self.dump:
            self.dump_pointclouds("find_transform_before.ply", self.source_pc, self.target_pc)
        if self.analyze:
            self.analyze_pointclouds("Before", self.source_pc, self.target_pc)
        self.aligner.set_reference_pointcloud(self.target_pc)
        self.aligner.add_tiled_pointcloud(self.source_pc)
        self.aligner.run(0)
        transform = self.aligner.get_result_transformation()
        self.result_pc = self.aligner.get_result_pointcloud()
        if self.dump:
            self.dump_pointclouds("find_transform_after.ply", self.result_pc, self.target_pc)
        if self.analyze:
            self.analyze_pointclouds("After", self.result_pc, self.target_pc)
        p_transform = transformation_topython(transform)
        print(f"Transform filter needed: --filter 'transform44({p_transform})'")

    def dump_pointclouds(self, filename: str, source: cwipc.cwipc_wrapper, target: cwipc.cwipc_wrapper):
        if self.verbose:
            print(f"Dumping point clouds to {filename}")
        colored_source = cwipc.cwipc_colormap(source, 0xFFFFFFFF, 0xAAFF0000)
        colored_target = cwipc.cwipc_colormap(target, 0xFFFFFFFF, 0xAA00FF00)
        combined = cwipc.cwipc_join(colored_source, colored_target)
        cwipc.cwipc_write(filename, combined)
        colored_source.free()
        colored_target.free()
        combined.free()
        
    def analyze_pointclouds(self, label : str, source: cwipc.cwipc_wrapper, target: cwipc.cwipc_wrapper):
        analyzer = RegistrationAnalyzer()
        analyzer.verbose = self.verbose
        analyzer.set_reference_pointcloud(target)
        analyzer.add_tiled_pointcloud(source)
        analyzer.run()
        results = analyzer.get_results()
        correspondence = results.minCorrespondence[0]
        sigma = results.minCorrespondenceSigma[0]
        count = results.minCorrespondenceCount[0]
        percentage = 100.0 * count / source.count()
        print(f"{label} alignment: correspondence: {correspondence}, sigma: {sigma}, count: {count}, percentage: {percentage:.2f}%")

def main():
    parser = argparse.ArgumentParser(description="Find transform between two pointclouds", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("target", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("--analyze", help="Print pre and post analysis of point cloud distance", action="store_true")
    parser.add_argument("--dump", help="Dump combined pre and post point clouds to files (color-coded)", action="store_true")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--debugpy", action="store_true", help="Wait for debugpy client to attach")
    args = parser.parse_args()
    if args.debugpy:
        import debugpy
        debugpy.listen(5678)
        print(f"{sys.argv[0]}: waiting for debugpy attach on 5678", flush=True)
        debugpy.wait_for_client()
        print(f"{sys.argv[0]}: debugger attached")        
    finder = TransformFinder(args)
    finder.load_source(args.source)
    finder.load_target(args.target)
    finder.run()
    
if __name__ == '__main__':
    main()
    
    
    
