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

acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];
static acomm_bus_metadata_type    *bus_map;

void acomm_generated_code_init(void)
{
    acomm_uint8 *p = (acomm_uint8*)bus_map;
    acomm_bus[bus_map->meta_busid].num = bus_map->meta_entrynum;
    acomm_bus[bus_map->meta_busid].meta = bus_map;
    acomm_bus[bus_map->meta_busid].comm_buffer_offset = (acomm_uint32*)&p[bus_map->meta_buffer_offset_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_size = (acomm_uint32*)&p[bus_map->meta_buffer_size_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_elmsize = (acomm_uint32*)&p[bus_map->meta_buffer_elmsize_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_type = (acomm_uint32*)&p[bus_map->meta_buffer_type_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer = (acomm_uint8*)&p[bus_map->data_data_soff];
    return;
}
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

static void acomm_read_entry(int index)
{
    acomm_rtype err;
#if 0
    if (index >= acomm_bus[bus_map->meta_busid].num) {
        fprintf(stderr, "ERROR:invalid index(%d). entry_num=%d\n", index,acomm_bus[bus_map->meta_busid].num );
        return;
    }
    printf("busid=%u\n", bus_map->meta_busid);
    printf("entrynum=%d\n", acomm_bus[bus_map->meta_busid].num);
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
    printf("entry[%d].type=%u\n", index, acomm_bus[bus_map->meta_busid].comm_buffer_type[index]);
    {
        int i;
        for (i = 0; i < acomm_bus[bus_map->meta_busid].num; i++) {
            printf("buffer_offset[%d]=%u\n", i, acomm_bus[bus_map->meta_busid].comm_buffer_offset[i]);
            printf("comm_buffer_size[%d]=%u\n", i, acomm_bus[bus_map->meta_busid].comm_buffer_size[i]);
            printf("comm_buffer_elmsize[%d]=%u\n", i, acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[i]);
            printf("comm_buffer_type[%d]=%u\n", i, acomm_bus[bus_map->meta_busid].comm_buffer_type[i]);
        }
    }
#endif

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
	int fd;
	int err;
	struct stat statbuf;
    int i;
    char *indexp;
    char *path;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <filepath> <index>\n", argv[0]);
        return 1;
    }
    path = (char*)argv[1];
    indexp = (char*)argv[2];
    i = atoi(indexp);

	fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Error: can not open file %s\n", path);
        return 1;
    }
    err = fstat(fd, &statbuf);
    if (err != 0) {
        fprintf(stderr, "Error: can not stat file %s\n", path);
        return 1;
    }
    bus_map = (acomm_bus_metadata_type*)mmap(NULL, statbuf.st_size, (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
    if (bus_map == NULL) {
        fprintf(stderr, "Error: can not mmap file %s\n", path);
        return 1;
    }
    //printf("path=%s index=%d\n", path, i);

    (void)athrill_comm_init();
    acomm_read_entry(i);
    close(fd);
    return 0;
}