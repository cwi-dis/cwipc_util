#include <stdio.h>

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
    int status = cwipc_write_debugdump(argv[2], obj, &message);
    if (status < 0) {
        fprintf(stderr, "%s: Cannot save pointcloud to cwipcdump: %s\n", argv[0], message);
        return 1;
    }

    return 0;
}

