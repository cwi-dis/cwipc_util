import sys
import os
import time
import threading
import time
import argparse
import traceback
import queue
import numpy as np
import cwipc
from PIL import Image
from ._scriptsupport import *
import struct

class FileWriter:
    def __init__(self, pcpattern=None, rgbpattern=None, depthpattern=None, skeletonpattern=None, verbose=False, queuesize=2, nodrop=False, flags=0):
        self.producer = None
        self.queue = queue.Queue(maxsize=queuesize)
        self.nodrop = nodrop
        self.verbose = verbose
        self.pcpattern = pcpattern
        self.rgbpattern = rgbpattern
        self.depthpattern = depthpattern
        self.skeletonpattern = skeletonpattern
        self.count = 0
        self.flags = flags
        self.error_encountered = False
        
    def set_producer(self, producer):
        self.producer = producer    
        
    def run(self):
        while (self.producer and self.producer.is_alive()) or not self.queue.empty():
            try:
                pc = self.queue.get(timeout=0.5)
                self.count = self.count + 1
                ok = self.save_pc(pc)
                pc.free()
                if not ok:
                    self.error_encountered = True
                    break
            except queue.Empty:
                pass
        if self.verbose:
            print(f"writer: stopped")
        return not self.error_encountered              
        
    def feed(self, pc):
        try:
            if self.nodrop:
                self.queue.put(pc)
            else:
                self.queue.put(pc, timeout=0.5)
            if self.verbose:
                print(f"writer: fed pointcloud {pc.timestamp()} to writer")
        except queue.Full:
            if self.verbose:
                print(f"writer: dropped pointcloud {pc.timestamp()}")
            pc.free()

    def save_pc(self, pc):
        """Save pointcloud"""
        if self.pcpattern:
            # Save pointcloud
            filename = self.pcpattern.format(timestamp=pc.timestamp(), count=self.count)
            ext = os.path.splitext(filename)[1].lower()
            if ext == '.ply':
                try:
                    cwipc.cwipc_write(filename, pc, self.flags)
                    if self.verbose:
                        print(f"writer: wrote pointcloud to {filename}")
                except cwipc.CwipcError as e:
                    print(f"writer: error: {e}")
                    self.error_encountered = True
            elif ext == '.cwipcdump':
                cwipc.cwipc_write_debugdump(filename, pc)
                if self.verbose:
                    print(f"writer: wrote pointcloud to {filename}")
            else:
                print(f"writer: Filetype unknown for pointcloud output: {filename}")
                return False
            # xxxjack could easily add compressed files, etc
        if self.rgbpattern or self.depthpattern or self.skeletonpattern:
            saved_any = False
            pc_auxdata = pc.access_auxiliary_data()
            for i in range(pc_auxdata.count()):
                aux_name = pc_auxdata.name(i)
                if aux_name.startswith('rgb'):
                    saved_any = self.save_auxdata('rgb', aux_name, pc, pc_auxdata.description(i), pc_auxdata.data(i), self.rgbpattern)
                if aux_name.startswith('depth'):
                    saved_any = self.save_auxdata('depth', aux_name, pc, pc_auxdata.description(i), pc_auxdata.data(i), self.depthpattern)
                if aux_name.startswith('skeleton'):
                    saved_any = self.save_auxdata_skeleton('skeleton', aux_name, pc, pc_auxdata.description(i), pc_auxdata.data(i), self.skeletonpattern)
            if not saved_any:
                print(f"writer: did not find any auxiliary data in pointcloud {pc.timestamp()}")
                #return False
        return True

    def save_auxdata(self, type, name, pc, description, data, pattern):
        filename = pattern.format(timestamp=pc.timestamp(), count=self.count, type=type, name=name)
        ext = os.path.splitext(filename)[1].lower()
        if ext == '.bin' or ext == '.raw':
            with open(filename, 'wb') as fp:
                fp.write(data)
            if self.verbose:
                print(f"writer: wrote {type} to {filename}")
        else:
            image = self.as_image(type, description, data)
            try:
                image.save(filename)
                if self.verbose:
                    print(f"writer: wrote {type} to {filename}")
            except ValueError:
                print(f"writer: Filetype {ext} unknown for {type} output: {filename}")
                return False
            except AttributeError:
                print("Couldn't save image {}".format(image))
        return True
    
    def save_auxdata_skeleton(self, type, name, pc, description, data, pattern):
        data_bytes = bytes(data)
        n_skeletons, n_joints = struct.unpack('II', data_bytes[:8])
        #print(f'n_skeletons= {n_skeletons} | n_joints= {n_joints}')
        if n_skeletons > 0:
            filename = pattern.format(timestamp=pc.timestamp(), count=self.count, type=type, name=name)
            ext = os.path.splitext(filename)[1].lower()
            if ext == '.txt':    
                with open(filename, "w") as f:
                    f.write('n_skeletons : '+str(n_skeletons)+'\n')
                    f.write('n_joints : '+str(n_joints)+'\n')
                    joint_byte_syze = 8 * 4 # 1 int and 7 floats = 8 elements of size 4 bytes
                    joints_size = joint_byte_syze * n_joints
                    total_size_joints = joints_size * n_skeletons
                    offset = 8
                    #print("Joints:")
                    for i in range(n_skeletons*n_joints):
                        joint_struct = struct.Struct('I 7f')
                        confidence, x, y, z, qw, qx, qy, qz = joint_struct.unpack_from(data,offset)
                        f.write(str((confidence, x, y, z, qw, qx, qy, qz))+'\n')
                        offset += joint_struct.size
                        #print((confidence, x, y, z, qw, qx, qy, qz))
                print(f"writer: wrote {type} to {filename}")
            else:
                print(f"Couldn't save skeleton. {ext} format not supported. Try txt")
                return False
        return True

    def parse_description(self, description):
        rv = {}
        fields = description.split(',')
        for f in fields:
            k, v = f.split('=')
            try:
                v = int(v)
            except ValueError:
                pass
            rv[k] = v
        return rv
        
    def bgra_to_rgb(self, bgra_image):
        bgra_image = bgra_image.convert("RGBA")
        data = np.array(bgra_image) 
        red, green, blue, alpha = data.T 
        data = np.array([blue, green, red, alpha])
        data = data.transpose()
        return Image.fromarray(data)
        
    def as_image(self, type, description, data):
        attrs = self.parse_description(description)
        #print(f"attrs: {attrs}")
        width = attrs['width']
        height = attrs['height']
        if 'bpp' in attrs:
            if attrs['bpp'] == 3:
                image_mode = 'RGB'
            elif attrs['bpp'] == 4:
                image_mode = 'RGBA'
            elif attrs['bpp'] == 2:
                image_mode = 'I;16'
            image = Image.frombytes(image_mode, (width, height), bytes(data), 'raw')
            return image
        elif 'format' in attrs:
            if attrs['format'] == 2:
                image_mode = 'RGB'
                bgra_image = Image.frombytes(image_mode, (width, height), bytes(data), 'raw')
                rgb_image = self.bgra_to_rgb(bgra_image)
            elif attrs['format'] == 3:
                image_mode = 'RGBA'
                bgra_image = Image.frombytes(image_mode, (width, height), bytes(data), 'raw')
                rgb_image = self.bgra_to_rgb(bgra_image)
            elif attrs['format'] == 4:
                image_mode = 'I;16'
                rgb_image = Image.frombytes(image_mode, (width, height), bytes(data), 'raw')
            return rgb_image
            
        
def main():
    SetupStackDumper()
    parser = ArgumentParser(description="Capture and save pointclouds")
    parser.add_argument("--nopointclouds", action="store_true", help="Don't save pointclouds")
    parser.add_argument("--cwipcdump", action="store_true", help="Save pointclouds as .cwipcdump (default: .ply)")
    parser.add_argument("--binary", action="store_true", help="Save pointclouds as binary .ply (default: ASCII .ply)")
    parser.add_argument("--rgb", action="store", metavar="EXT", help="Save RGB auxiliary data as images of type EXT")
    parser.add_argument("--depth", action="store", metavar="EXT", help="Save depth auxiliary data as images of type EXT")
    parser.add_argument("--skeleton", action="store", metavar="EXT", help="Save skeleton auxiliary data as files of type EXT")
    parser.add_argument("--fpattern", action="store", metavar="VAR", default="count:04d", help="Construct filenames using VAR, which can be count or timestamp (default)")
    parser.add_argument("--incore", action="store_true", help="Attempt to store all captures, at the expense of horrendous memory usage. Requires --count")
    parser.add_argument("outputdir", action="store", help="Save output files in this directory")

    args = parser.parse_args()
    #
    # Create source
    #
    sourceFactory, source_name = cwipc_genericsource_factory(args)
    source = sourceFactory()
    #
    # Determine which output formats we want, set output filename pattern
    # and ensure the requested data is included by the capturer
    #
    if args.nopointclouds:
        pcpattern = None
    elif args.cwipcdump:
        pcpattern = f"{args.outputdir}/pointcloud-{{{args.fpattern}}}.cwipcdump"
    else:
        pcpattern = f"{args.outputdir}/pointcloud-{{{args.fpattern}}}.ply"
    rgbpattern = None
    if args.rgb:
        rgbpattern = f"{args.outputdir}/{{name}}-{{{args.fpattern}}}.{args.rgb}"
        source.request_auxiliary_data("rgb")
    depthpattern = None
    if args.depth:
        depthpattern = f"{args.outputdir}/{{name}}-{{{args.fpattern}}}.{args.depth}"
        source.request_auxiliary_data("depth")
    skeletonpattern = None
    if args.skeleton:
        skeletonpattern = f"{args.outputdir}/{{name}}-{{{args.fpattern}}}.{args.skeleton}"
        source.request_auxiliary_data("skeleton")
        
    kwargs = {}
    if args.incore: # Attempt realtime capturing by storing all frames incore
        if args.count:
            kwargs['queuesize'] = args.count
        else:
            kwargs['queuesize'] = 2000 # xxxnacho. up to 2000 frames, but need to find a solution for k4aoffline case
    if args.nodrop:
        kwargs['nodrop'] = True
    if args.binary:
        kwargs['flags'] = cwipc.CWIPC_FLAGS_BINARY
    writer = FileWriter(
        pcpattern=pcpattern,
        rgbpattern=rgbpattern,
        depthpattern=depthpattern,
        skeletonpattern=skeletonpattern,
        verbose=args.verbose,
        **kwargs
        )

    sourceServer = SourceServer(source, writer, count=args.count, inpoint=args.inpoint, outpoint=args.outpoint, verbose=args.verbose, source_name=source_name)
    sourceThread = threading.Thread(target=sourceServer.run, args=())
    writer.set_producer(sourceThread)

    #
    # Run everything
    #
    ok = False
    try:
        sourceThread.start()

        if not args.incore:
            ok = writer.run()
            
        sourceThread.join()
        if args.incore:
            ok = writer.run()
    except KeyboardInterrupt:
        print("Interrupted.")
        sourceServer.stop()
    except:
        traceback.print_exc()
    
    #
    # It is safe to call join (or stop) multiple times, so we ensure to cleanup
    #
    sourceServer.stop()
    sourceThread.join()
    sourceServer.statistics()
    if not ok:
        sys.exit(1)
    
if __name__ == '__main__':
    main()
    
    
    
