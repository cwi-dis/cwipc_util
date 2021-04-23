#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "cwipc_util/api.h"

int main(int argc, char** argv)
{
    char *message = NULL;
    if (argc != 3) {
        fprintf(stderr, "Usage: %s pointcloudfile.ply pointcloudfile.cwipcdump\n", argv[0]);
        return 2;
    }
    //
    // Read pointcloud file
    //
    cwipc *obj = cwipc_read(argv[1], 0, &message, CWIPC_API_VERSION);
    if (obj == NULL) {
        fprintf(stderr, "%s: Cannot read pointcloud: %s\n", argv[0], message);
        return 1;
    }
    //
    // Save
    //
    if (strcmp(argv[2], "-") == 0) {
        // copy-uncompressed in stead of save (for performance testing)
        size_t nbytes = cwipc_get_uncompressed_size(obj);
        struct cwipc_point *points = (struct cwipc_point *)malloc(nbytes);
        if (points == NULL) {
            fprintf(stderr, "%s: out of memory\n", argv[0]);
            return 1;
        }
        cwipc_copy_uncompressed(obj, points, nbytes);
        fprintf(stderr, "%s: Skipping save\n", argv[0]);
    } else {

        int status = cwipc_write_debugdump(argv[2], obj, &message);
        if (status < 0) {
            fprintf(stderr, "%s: Cannot save pointcloud to cwipcdump: %s\n", argv[0], message);
            return 1;
        }
    }
    return 0;
}

