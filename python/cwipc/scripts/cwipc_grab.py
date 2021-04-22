import sys
import os
import time
import threading
import time
import argparse
import traceback
import queue
import cwipc
from ._scriptsupport import *

class FileWriter:
    def __init__(self, pcpattern=None, rgbpattern=None, depthpattern=None, skeletonpattern=None, verbose=False, queuesize=2):
        self.producer = None
        self.queue = queue.Queue(maxsize=queuesize)
        self.verbose = verbose
        self.pcpattern = pcpattern
        self.rgbpattern = rgbpattern
        self.depthpattern = depthpattern
        self.skeletonpattern = skeletonpattern
        self.count = 0
        
    def set_producer(self, producer):
        self.producer = producer    
        
    def run(self):
        while (self.producer and self.producer.is_alive()) or not self.queue.empty():
            try:
                pc = self.queue.get()
                self.count = self.count + 1
                ok = self.save_pc(pc)
                pc.free()
                if not ok: break
            except queue.Empty:
                pass
        if self.verbose:
            print(f"writer: stopped")
            print(f"xxxjack producer.is_alive: {self.producer.is_alive()}")
                
        
    def feed(self, pc):
        try:
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
                cwipc.cwipc_write(filename, pc)
                if self.verbose:
                    print(f"writer: wrote pointcloud to {filename}")
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
                    # xxxjack skeleton probably needs to be saved using different method
                    saved_any = self.save_auxdata('skeleton', aux_name, pc, pc_auxdata.description(i), pc_auxdata.data(i), self.skeletonpattern)
            if not saved_any:
                print(f"writer: did not find any auxiliary data in pointcloud {pc.timestamp()}")
                return False
        return True

    def save_auxdata(self, type, name, pc, description, data, pattern):
        filename = pattern.format(timestamp=pc.timestamp(), count=self.count, type=type, name=name)
        ext = os.path.splitext(filename)[1].lower()
        if 'ext' == '.bin':
            with open(filename, 'wb') as fp:
                fp.write(data)
            if self.verbose:
                print(f"writer: wrote {type} to {filename}")
        else:
            print(f"writer: Filetype unknown for {type} output: {filename}")
            return False
        return True

    
def main():
    SetupStackDumper()
    parser = ArgumentParser(description="Capture and save pointclouds")
    parser.add_argument("--nopointclouds", action="store_true", help="Don't save pointclouds")
    parser.add_argument("--cwipcdump", action="store_true", help="Save pointclouds as .cwipcdump (default: .ply)")
    parser.add_argument("--rgb", action="store", metavar="EXT", help="Save RGB auxiliary data as images of type EXT")
    parser.add_argument("--depth", action="store", metavar="EXT", help="Save depth auxiliary data as images of type EXT")
    parser.add_argument("--skeleton", action="store", metavar="EXT", help="Save skeleton auxiliary data as files of type EXT")
    parser.add_argument("--fpattern", action="store", metavar="VAR", default="timestamp", help="Construct filenames using VAR, which can be count or timestamp (default)")
    parser.add_argument("--all", action="store_true", help="Attempt to store all captures, at the expense of horrendous memory usage. Requires --count")
    parser.add_argument("outputdir", action="store", help="Save output files in this directory")

    args = parser.parse_args()
    #
    # Create source
    #
    sourceFactory, _ = cwipc_genericsource_factory(args)
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
        rgbpattern = f"{args.outputdir}/{{type}}-{{{args.fpattern}}}.{{args.rgb}}"
        source.request_auxiliary_data("rgb")
    depthpattern = None
    if args.depth:
        depthpattern = f"{args.outputdir}/{{type}}-{{{args.fpattern}}}.{{args.depth}}"
        source.request_auxiliary_data("depth")
    skeletonpattern = None
    if args.skeleton:
        skeletonpattern = f"{args.outputdir}/{{type}}-{{{args.fpattern}}}.{{args.skeleton}}"
        source.request_auxiliary_data("skeleton")
    
    if args.all:
        kwargs = {'queuesize' : args.count}
    else:
        kwargs = {}
    writer = FileWriter(
        pcpattern=pcpattern,
        rgbpattern=rgbpattern,
        depthpattern=depthpattern,
        skeletonpattern=skeletonpattern,
        verbose=args.verbose,
        **kwargs
        )

    sourceServer = SourceServer(source, writer, count=args.count, verbose=args.verbose)
    sourceThread = threading.Thread(target=sourceServer.run, args=())
    writer.set_producer(sourceThread)

    #
    # Run everything
    #
    try:
        sourceThread.start()

        writer.run()
            
        sourceThread.join()
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
    
if __name__ == '__main__':
    main()
    
    
    
