import sys
import os
import argparse
import cwipc
from cwipc.scripts.cwipc_calibrate.pointcloud import Pointcloud

colors_list = [] #to store the color palette
colors_list.append([0,0,1])
colors_list.append([0,1,0])
colors_list.append([1,0,0])
colors_list.append([0,1,1])
colors_list.append([1,0,1])
colors_list.append([1,1,0])
colors_list.append([1,1,1])

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", action="store", metavar="input_file.ply", help="Input ply file to read")
    parser.add_argument("-o", action="store", metavar="output_file.ply", help="Output ply file to generate")
    args = parser.parse_args()
    #
    # Create source
    #
    if args.i and args.o:
        source_pc = cwipc.cwipc_read(args.i, 0)
        pcc = Pointcloud.from_cwipc(source_pc)
        pc_split = pcc.split()
        print("Current pointcloud has ",len(pc_split), " tiles")
        for i in range(len(pc_split)):
            pc_split[i] = Pointcloud.from_o3d(pc_split[i].get_o3d().paint_uniform_color(colors_list[i]))
        pc_out = Pointcloud.from_join(pc_split)
        pc_out.save(args.o)
        print("Output colorized pointcloud saved in ",args.o)
    else:
        print("ERROR. Usage: cwipc_tilecolor -i input.ply -o output.ply")
        return
    
if __name__ == '__main__':
    main()
    
    
    
