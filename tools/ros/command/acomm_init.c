#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/file.h>

#include "athrill_comm.h"
#include "athrill_comm_config.h"
#include "athrill_comm_generated_config.h"

acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];
static acomm_bus_metadata_type    *bus_map = NULL;
static int fd = -1;

void acomm_generated_code_init(void)
{
    acomm_uint8 *p = (acomm_uint8*)bus_map;
    acomm_bus[bus_map->meta_busid].num = bus_map->meta_entrynum;
    acomm_bus[bus_map->meta_busid].meta = bus_map;
    acomm_bus[bus_map->meta_busid].comm_buffer_offset = (acomm_uint32*)&p[bus_map->meta_buffer_offset_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_size = (acomm_uint32*)&p[bus_map->meta_buffer_size_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_elmsize = (acomm_uint32*)&p[bus_map->meta_buffer_elmsize_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_earraysize = (acomm_uint32*)&p[bus_map->meta_buffer_earraysize_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer_type = (acomm_uint32*)&p[bus_map->meta_buffer_type_soff];
    acomm_bus[bus_map->meta_busid].comm_buffer = (acomm_uint8*)&p[bus_map->data_data_soff];
    return;
}

acomm_bus_metadata_type *acomm_open(char *path)
{
    int err;
    struct stat statbuf;

	fd = open(path, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Error: can not open file %s\n", path);
        return NULL;
    }
    err = fstat(fd, &statbuf);
    if (err != 0) {
        fprintf(stderr, "Error: can not stat file %s\n", path);
        return NULL;
    }
    bus_map = (acomm_bus_metadata_type*)mmap(NULL, statbuf.st_size, (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
    if (bus_map == NULL) {
        fprintf(stderr, "Error: can not mmap file %s\n", path);
        return NULL;
    }

    (void)athrill_comm_init();

    return bus_map;
}

void acomm_close(acomm_bus_metadata_type *p)
{
    if (fd >= 0) {
        close(fd);
        fd = -1;
    }
    bus_map = NULL;
    return;
}

void acomm_lock(acomm_busid busid)
{
    if (fd >= 0) {
        flock(fd, LOCK_EX);
    }
    return;
}

void acomm_unlock(acomm_busid busid)
{
    if (fd >= 0) {
        flock(fd, LOCK_UN);
    }
    return;
}
