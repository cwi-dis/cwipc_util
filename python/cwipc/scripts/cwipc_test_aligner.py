import sys
from typing import Optional
import argparse
import traceback
import cwipc
from cwipc.registration.multicamera import DEFAULT_MULTICAMERA_ALGORITHM
from cwipc.registration.fine import DEFAULT_FINE_ALIGNMENT_ALGORITHM
from cwipc.registration.analyze import DEFAULT_ANALYZER_ALGORITHM, AnalysisResults
from cwipc.registration.util import transformation_topython, cwipc_tilefilter_masked
from cwipc.registration.plot import Plotter

class AlignmentFinder:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.plot = args.plot
        self.verbose = args.verbose
#        self.correspondence = args.correspondence
        self.input_pc : Optional[cwipc.cwipc_wrapper] = None
        self.result_pc : Optional[cwipc.cwipc_wrapper] = None
        self.multi_aligner = DEFAULT_MULTICAMERA_ALGORITHM()
        self.multi_aligner.verbose = self.verbose
        self.multi_aligner.set_analyzer_class(DEFAULT_ANALYZER_ALGORITHM)
        self.multi_aligner.set_aligner_class(DEFAULT_FINE_ALIGNMENT_ALGORITHM)
        self.multi_aligner.show_plot = self.plot
            
    def load_input(self, source: str):
        pc = cwipc.cwipc_read(source, 0)
        self.input_pc = pc
    
    def save_output(self, filename: str):
        assert self.output_pc
        cwipc.cwipc_write(filename, self.output_pc)
        
    def run(self):
        assert self.input_pc
        self.multi_aligner.set_tiled_pointcloud(self.input_pc)
        self.multi_aligner.run()
        self.output_pc = self.multi_aligner.get_result_pointcloud_full()
        transformations = self.multi_aligner.get_result_transformations()
        for i in range(len(transformations)):
            print(f"Tile {i} transformation: {transformation_topython(transformations[i])}")

    
def main():
    parser = argparse.ArgumentParser(description="Find transform between two pointclouds", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("output", help="Point cloud, as .ply or .cwipc file")
    parser.add_argument("--plot", action="store_true", help="Plot analysis distance distribution")
#    parser.add_argument("--correspondence", type=float, metavar="FLOAT", default=-1, help="Correspondence threshold for alignment (default: use analysis result)")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--debugpy", action="store_true", help="Wait for debugpy client to attach")
    args = parser.parse_args()
    if args.debugpy:
        import debugpy
        debugpy.listen(5678)
        print(f"{sys.argv[0]}: waiting for debugpy attach on 5678", flush=True)
        debugpy.wait_for_client()
        print(f"{sys.argv[0]}: debugger attached")
    finder = AlignmentFinder(args)
    finder.load_input(args.input)
    finder.run()
    finder.save_output(args.output)
    
if __name__ == '__main__':
    main()
    
    
    
