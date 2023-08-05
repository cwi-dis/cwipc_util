import sys
import cwipc

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} pointcloudfile.cwipcdump pointcloudfile.ply", file=sys.stderr)
        sys.exit(2)

    pc = cwipc.cwipc_read_debugdump(sys.argv[1])

    cwipc.cwipc_write(sys.argv[2], pc)

    pc.free()

if __name__ == '__main__':
    main()
    