import sys
from typing import Optional
import argparse
import traceback
import cwipc
from cwipc.registration.abstract import AnalysisResults
from cwipc.registration.fine import RegistrationComputer_ICP_Point2Point
from cwipc.registration.analyze import RegistrationAnalyzer, RegistrationAnalyzerIgnoreNearest
from cwipc.registration.util import transformation_topython, get_tiles_used
from cwipc.registration.plot import Plotter

class AnalyzePointCloud:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.verbose = args.verbose
        self.source_pc : Optional[cwipc.cwipc_wrapper] = None
        self.result_pc : Optional[cwipc.cwipc_wrapper] = None
        if args.toself:
            self.analyzer_algorithm = RegistrationAnalyzerIgnoreNearest
        else:
            self.analyzer_algorithm = RegistrationAnalyzer
            
    def load_source(self, source: str):
        pc = cwipc.cwipc_read(source, 0)
        self.source_pc = pc
    
    def run(self):
        assert self.source_pc
        tiles = get_tiles_used(self.source_pc)
        if (not tiles or len(tiles) <= 1) and not self.args.toself:
            print(f"Source point cloud {self.source_pc} has no tiles or only one tile, cannot analyze registration.")
            return
        print(f"Tiles used in source: {tiles}")
        allResults = []
        if self.args.toself:
            title = "Distance between adjacent points in the same tile"
        else:
            title = "Distance between each tile and all others"
        for sourcetile in tiles:
            if not self.args.toself:
                targettile = 255 - sourcetile
            else:
                targettile = sourcetile
            results = self.analyze_pointclouds(self.source_pc, sourcetile, targettile)
            allResults.append(results)
        if self.args.plot:
            plotter = Plotter(title=title)
            plotter.set_results(allResults)
            plotter.plot(show=True)
 
    def analyze_pointclouds(self, source: cwipc.cwipc_wrapper, sourcetile : int, targettile : int) -> AnalysisResults:
        analyzer = self.analyzer_algorithm()
        if self.args.toself:
            analyzer.set_ignore_nearest(self.args.nth)
        analyzer.verbose = self.verbose
        analyzer.set_source_pointcloud(source, sourcetile)
        analyzer.set_reference_pointcloud(source, targettile)
        analyzer.run()
        results = analyzer.get_results()
        correspondence = results.minCorrespondence
        sigma = results.minCorrespondenceSigma
        count = results.minCorrespondenceCount
        assert analyzer.source_pointcloud is not None
        percentage = 100.0 * count / analyzer.source_pointcloud.count()
        if self.args.toself:
            label = f"{sourcetile:#x} self, nth={self.args.nth}"
        else:
            label = f"{sourcetile:#x} to {targettile:#x}"
        print(f"Alignment {label}: correspondence: {correspondence:.6f}, sigma: {sigma:.6f}, count: {count}, percentage: {percentage:.2f}%")
        return results

def main():
    parser = argparse.ArgumentParser(description="Find registration of a tiled point cloud", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("source", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("--plot", action="store_true", help="Plot analysis distance distribution")
    parser.add_argument("--toself", action="store_true", help="Analyze self-registration (source and target tile the same), for judging capture quality")
    parser.add_argument("--nth", type=int, default=1, metavar="NTH", help="For --toself, use the NTH closest point to each point in the same tile (default: 1, i.e. nearest point)")
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
    
    
    
