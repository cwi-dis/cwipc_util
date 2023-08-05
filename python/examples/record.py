import sys
import cwipc

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} count dir", file=sys.stderr)
        print("Capture pointclouds store them in a directory", file=sys.stderr)
        sys.exit(2)
    
    count = int(sys.argv[1])
    outdir = sys.argv[2]

    generator = cwipc.cwipc_capturer()
   
    ok = True
    for i in range(count):
        if not generator.available(True):
            print("No pointcloud available?")
            break
        pc = generator.get()
        assert pc
        cwipc.cwipc_write_debugdump(f"{outdir}/pointcloud-{i}.cwipcdump", pc)
        pc.free()

    generator.free()

if __name__ == '__main__':
    main()
    