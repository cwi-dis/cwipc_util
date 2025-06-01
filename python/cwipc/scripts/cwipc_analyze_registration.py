import sys
from typing import Optional
import argparse
import traceback
import cwipc
from cwipc.registration.fine import RegistrationComputer_ICP_Point2Point
from cwipc.registration.analyze import RegistrationAnalyzer
from cwipc.registration.util import transformation_topython, get_tiles_used
from cwipc.registration.plot import Plotter

class AnalyzePointCloud:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.plot = args.plot
        self.verbose = args.verbose
        self.source_pc : Optional[cwipc.cwipc_wrapper] = None
        self.result_pc : Optional[cwipc.cwipc_wrapper] = None
            

    def load_source(self, source: str):
        pc = cwipc.cwipc_read(source, 0)
        self.source_pc = pc

    
    def run(self):
        assert self.source_pc
        self.analyze_pointclouds(self.source_pc, self.args.sourcetile, self.args.targettile)
        
    def analyze_pointclouds(self, source: cwipc.cwipc_wrapper, sourcetile : int, targettile : int):
        print(f"Tiles used in source: {get_tiles_used(source)}")
        analyzer = RegistrationAnalyzer()
        analyzer.verbose = self.verbose
        analyzer.set_source_pointcloud(source, sourcetile)
        analyzer.set_reference_pointcloud(source, targettile)
        analyzer.run()
        results = analyzer.get_results()
        correspondence = results.minCorrespondence
        sigma = results.minCorrespondenceSigma
        count = results.minCorrespondenceCount
        percentage = 100.0 * count / source.count()
        label = f"{sourcetile:#x} to {targettile:#x}"
        print(f"Alignment {label}: correspondence: {correspondence}, sigma: {sigma}, count: {count}, percentage: {percentage:.2f}%")
        if self.plot:
            plotter = Plotter(title=label)
            plotter.set_results([results])
            plotter.plot(show=True)
def main():
    parser = argparse.ArgumentParser(description="Find registration of a tiled point cloud", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("--plot", action="store_true", help="Plot analysis distance distribution")
    parser.add_argument("--sourcetile", type=int, metavar="NUM", default=0, help="Filter source point cloud to tile NUM before alignment")
    parser.add_argument("--targettile", type=int, metavar="NUM", default=0, help="Filter target point cloud to tile NUM before alignment")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--debugpy", action="store_true", help="Wait for debugpy client to attach")
    args = parser.parse_args()
    if args.debugpy:
        import debugpy
        debugpy.listen(5678)
        print(f"{sys.argv[0]}: waiting for debugpy attach on 5678", flush=True)
        debugpy.wait_for_client()
        print(f"{sys.argv[0]}: debugger attached")
    finder = AnalyzePointCloud(args)
    finder.load_source(args.source)
    finder.run()
    
if __name__ == '__main__':
    main()
    
    
    
