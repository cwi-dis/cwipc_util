import sys
import os
import argparse
from .. import cwipc_read
from .cwipc_calibrate.pointcloud import Pointcloud

colors_list_tilenum = [] #to store the color palette
colors_list_tilenum.append([0,0,0])
colors_list_tilenum.append([0,0,1])
colors_list_tilenum.append([0,1,0])
colors_list_tilenum.append([1,0,0])
colors_list_tilenum.append([0,1,1])
colors_list_tilenum.append([1,0,1])
colors_list_tilenum.append([1,1,0])
colors_list_tilenum.append([1,1,1])

COL_0 = (0,0,0)
COL_1 = (1,1,1)
COL_2 = (1, 0, 0)
COL_3 = (0, 1, 0)
COL_4 = (0, 1, 0)

colors_list_ncam = []
colors_list_ncam.append(COL_0)
colors_list_ncam.append(COL_1)
colors_list_ncam.append(COL_1)
colors_list_ncam.append(COL_2)
colors_list_ncam.append(COL_1)
colors_list_ncam.append(COL_2)
colors_list_ncam.append(COL_2)
colors_list_ncam.append(COL_3)
colors_list_ncam.append(COL_1)
colors_list_ncam.append(COL_2)
colors_list_ncam.append(COL_2)
colors_list_ncam.append(COL_3)
colors_list_ncam.append(COL_2)
colors_list_ncam.append(COL_3)
colors_list_ncam.append(COL_3)
colors_list_ncam.append(COL_4)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", action="store", metavar="input_file.ply", help="Input ply file to read")
    parser.add_argument("-o", action="store", metavar="output_file.ply", help="Output ply file to generate")
    parser.add_argument("-m", action="store_true", help="Use camera-count map in stead of tile-number map")
    args = parser.parse_args()
    colors_list = colors_list_tilenum
    if args.m:
        colors_list = colors_list_ncam
    #
    # Create source
    #
    if args.i and args.o:
        source_pc = cwipc_read(args.i, 0)
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
    
    
    
