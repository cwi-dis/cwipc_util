import sys
from typing import Optional
import argparse
import traceback
import numpy as np
import cwipc
from cwipc.registration.fine import RegistrationComputer_ICP_Point2Point
from cwipc.registration.analyze import RegistrationAnalyzer
from cwipc.registration.util import transformation_topython, transformation_identity, cwipc_transform
from cwipc.registration.plot import Plotter
from cwipc.filters.simulatecams import SimulatecamsFilter
from cwipc.filters.noise import NoiseFilter

class AnalysisTestCreator:
    def __init__(self, args : argparse.Namespace):
        self.args = args
        self.verbose = args.verbose
        self.noise = args.noise
        self.ncamera = args.ncamera
        self.per_camera_movement = args.move
        self.input_pc : Optional[cwipc.cwipc_wrapper] = None
        self.output_pc : Optional[cwipc.cwipc_wrapper] = None
            

    def load_input(self, source: str):
        pc = cwipc.cwipc_read(source, 0)
        self.input_pc = pc

    def save_output(self, target: str):
        assert self.output_pc
        cwipc.cwipc_write(target, self.output_pc)
       
    def run(self):
        assert self.input_pc
        sim_filter = SimulatecamsFilter(self.args.ncamera)
        tiled_pc = sim_filter.filter(self.input_pc)
        if self.verbose:
            print(f"Input point cloud tiled into {self.args.ncamera} cameras, {tiled_pc.count()} points in total.")
        per_tile_pcs = []
        for camnum in range(self.args.ncamera):
            tilemask = 1 << camnum
            per_tile_pc = cwipc.cwipc_tilefilter(tiled_pc, tilemask)
            if self.verbose:
                print(f"Tile {camnum} has {per_tile_pc.count()} points")
            if camnum < len(self.per_camera_movement) and self.per_camera_movement[camnum] > 0:
                movement = self.per_camera_movement[camnum]
                random_angle = np.random.uniform(0, 2 * np.pi)
                delta_x = movement * np.cos(random_angle)
                delta_z = movement * np.sin(random_angle)
                transform = transformation_identity()
                transform[0, 3] = delta_x
                transform[2, 3] = delta_z
                if self.verbose:
                    print(f"Moving tile {camnum} by {transform}")
                new_per_tile_pc = cwipc_transform(per_tile_pc, transform)
                per_tile_pc.free()
                per_tile_pc = new_per_tile_pc
            per_tile_pcs.append(per_tile_pc)
        cwipc_joined_pc = per_tile_pcs.pop()
        while per_tile_pcs:
            next_pc = per_tile_pcs.pop()
            cwipc_joined_pc = cwipc.cwipc_join(cwipc_joined_pc, next_pc)
            next_pc.free()
        if self.noise > 0:
            if self.verbose:
                print(f"Adding noise of {self.noise} meters to the point cloud")
            noise_filter = NoiseFilter(self.noise)
            cwipc_joined_pc = noise_filter.filter(cwipc_joined_pc)
        self.output_pc = cwipc_joined_pc
       
def main():
    parser = argparse.ArgumentParser(description="Create a point cloud for testing analysis and registration", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("input", help="Input point cloud .ply file")
    parser.add_argument("output", help="Output point cloud .ply file")
    parser.add_argument("--ncamera", type=int, metavar="NUM", default=1, help="Number of cameras to simulate")
    parser.add_argument("--move", type=float, action="append", metavar="D", help="Distance to move a tile (in meters) in the Y=0 plane")
    parser.add_argument("--noise", type=float, metavar="DIST", default=0.0, help="Add noise to each point (in meters)")
    parser.add_argument("--verbose", action="store_true", help="Verbose output")
    parser.add_argument("--debugpy", action="store_true", help="Wait for debugpy client to attach")
    args = parser.parse_args()
    if args.debugpy:
        import debugpy
        debugpy.listen(5678)
        print(f"{sys.argv[0]}: waiting for debugpy attach on 5678", flush=True)
        debugpy.wait_for_client()
        print(f"{sys.argv[0]}: debugger attached")
    creator = AnalysisTestCreator(args)
    creator.load_input(args.input)
    creator.run()
    creator.save_output(args.output)
    
if __name__ == '__main__':
    main()
    
    
    
