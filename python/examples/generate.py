import sys
import cwipc

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} count directory", file=sys.stderr)
        print("Creates COUNT synthetic pointclouds and stores the PLY files in the given directory")
        sys.exit(2)
    count = int(sys.argv[1])
    directory = sys.argv[2]

    generator = cwipc.cwipc_synthetic()

    for i in range(count):
        pc = generator.get()
        assert pc
        cwipc.cwipc_write(f"{directory}/pointcloud-{i}.ply", pc)
        pc.free()

    generator.free()

if __name__ == '__main__':
    main()
    