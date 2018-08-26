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

static void acomm_stat_entry_single(acomm_bus_metadata_type *bus_map, int index)
{
    printf("comm_buffer_offset[%d]=%u\n",        index, acomm_bus[bus_map->meta_busid].comm_buffer_offset[index]);
    printf("comm_buffer_size[%d]=%u\n",     index, acomm_bus[bus_map->meta_busid].comm_buffer_size[index]);
    printf("comm_buffer_elmsize[%d]=%u\n",  index, acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[index]);
    if (acomm_bus[bus_map->meta_busid].comm_buffer_type[index] == AcommDataType_Primitive) {
        printf("comm_buffer_type[%d]=%s\n",     index, "PrimitiveType");
    }
    else {
        int off;
        acomm_queue_type *entry;
        printf("comm_buffer_type[%d]=%s\n",     index, "QueueType");
        off = acomm_bus[bus_map->meta_busid].comm_buffer_offset[index] - bus_map->data_data_soff;
        entry = (acomm_queue_type*)&acomm_bus[bus_map->meta_busid].comm_buffer[off];
        printf("entry->elmsize=%u\n", entry->elmsize);
        printf("entry->maxlen=%u\n", entry->maxlen);
        printf("entry->len=%u\n", entry->len);
        printf("entry->roff=%u\n", entry->roff);
        printf("entry->woff=%u\n", entry->woff);
    }

    return;
}

static void acomm_stat_entry(acomm_bus_metadata_type *bus_map, int index)
{
    if (index < 0) {
        int i;
        for (i = 0; i < acomm_bus[bus_map->meta_busid].num; i++) {
            acomm_stat_entry_single(bus_map, i);
        }
        return;
    }
    if (index >= acomm_bus[bus_map->meta_busid].num) {
        fprintf(stderr, "ERROR:invalid index(%d). entry_num=%d\n", index,acomm_bus[bus_map->meta_busid].num );
        return;
    }
    acomm_stat_entry_single(bus_map, index);
    return;
}


static void acomm_stat(acomm_bus_metadata_type *bus_map)
{
    printf("meta_version=0x%x\n", bus_map->meta_version);
    printf("meta_magicno=0x%x\n", bus_map->meta_magicno);
    printf("meta_busid=%u\n", bus_map->meta_busid);
    printf("meta_entrynum=%d\n", acomm_bus[bus_map->meta_busid].num);
    printf("meta_buffer_offset_soff=%u\n", bus_map->meta_buffer_offset_soff);
    printf("meta_buffer_offset_size=%u\n", bus_map->meta_buffer_offset_size);
    printf("meta_buffer_size_soff=%u\n", bus_map->meta_buffer_size_soff);
    printf("meta_buffer_size_size=%u\n", bus_map->meta_buffer_size_size);
    printf("meta_buffer_elmsize_soff=%u\n", bus_map->meta_buffer_elmsize_soff);
    printf("meta_buffer_elmsize_size=%u\n", bus_map->meta_buffer_elmsize_size);
    printf("meta_buffer_type_soff=%u\n", bus_map->meta_buffer_type_soff);
    printf("meta_buffer_type_size=%u\n", bus_map->meta_buffer_type_size);
    printf("data_data_soff=%u\n", bus_map->data_data_soff);
    printf("data_data_size=%u\n", bus_map->data_data_size);

    return;
}
int main(int argc, const char *argv[])
{
    int i = -1;
    char *indexp = NULL;
    char *path;
    acomm_bus_metadata_type *p;

    if ((argc != 3) && (argc != 2)) {
        fprintf(stderr, "Usage: %s <filepath> [<index>]\n", argv[0]);
        return 1;
    }
    path = (char*)argv[1];
    if (argc == 3) {
        indexp = (char*)argv[2];
        i = atoi(indexp);
    }

    p = acomm_open(path);
    if (p == NULL) {
        fprintf(stderr, "Error: acomm_init() %s\n", path);
        return 1;
    }
    if (indexp == NULL) {
        acomm_stat(p);
    }
    else {
        acomm_stat_entry(p, i);
    }
    acomm_close(p);
    return 0;
}
