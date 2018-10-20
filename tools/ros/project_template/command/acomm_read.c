#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include "athrill_comm.h"
#include "athrill_comm_config.h"
#include "athrill_comm_generated_config.h"
#include "acomm_init.h"

static acomm_uint8 retdata[4096];

static void acomm_show_data(acomm_uint8 *datap, acomm_uint32 size)
{
    int i;
    switch (size) {
    case 1:
        printf("(uint8) %u\n", datap[0]);
        break;
    case 2:
        printf("(uint16) %u\n", *((acomm_uint16*)&datap[0]));
        break;
    case 4:
        printf("(uint32) %u\n", *((acomm_uint32*)&datap[0]));
        break;
    default:
        for (i = 0; i < size; i++) {
            printf("%d: (uint8) %u\n", i, datap[i]);
        }
        break;
    }
    return;
}

static void acomm_read_entry(acomm_bus_metadata_type *bus_map, int index)
{
    acomm_rtype err;
    memset(retdata, 0, sizeof(retdata));
    if (acomm_bus[bus_map->meta_busid].comm_buffer_type[index] == AcommDataType_Queue) {
        err = athrill_comm_peek(bus_map->meta_busid, index, &retdata[0], acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[index]);
    }
    else {
        err = athrill_comm_read(bus_map->meta_busid, index, &retdata[0], acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[index]);
    }
    if (err != ACOMM_E_OK) {
        fprintf(stderr, "ERROR:can not read index(%d). err=%d\n", index, err);
    }
    else {
        acomm_show_data(retdata, acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[index]);
    }
    return;
}

int main(int argc, const char *argv[])
{
    int i;
    char *indexp;
    char *path;
    acomm_bus_metadata_type *p;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <filepath> <index>\n", argv[0]);
        return 1;
    }
    path = (char*)argv[1];
    indexp = (char*)argv[2];
    i = atoi(indexp);

    p = acomm_open(path);
    if (p == NULL) {
        fprintf(stderr, "Error: acomm_init() %s\n", path);
        return 1;
    }
    acomm_read_entry(p, i);
    acomm_close(p);
    return 0;
}
