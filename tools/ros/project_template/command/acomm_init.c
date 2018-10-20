#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/file.h>
#include <pthread.h>

#include "athrill_comm.h"
#include "athrill_comm_config.h"
#include "athrill_comm_generated_config.h"

acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];
static acomm_bus_metadata_type    *bus_map[ATHRILL_COMM_CONFIG_BUS_NUM];
static int bus_fd[ATHRILL_COMM_CONFIG_BUS_NUM];
static pthread_mutex_t acomm_mutex[ATHRILL_COMM_CONFIG_BUS_NUM];

static void acomm_generated_code_init_common(acomm_uint32 busid)
{
    acomm_bus_metadata_type *tmp_bus_map = bus_map[busid];
    acomm_uint8 *p = (acomm_uint8*)bus_map[busid];
    acomm_bus[busid].num = tmp_bus_map->meta_entrynum;
    acomm_bus[busid].meta = tmp_bus_map;
    acomm_bus[busid].comm_buffer_offset = (acomm_uint32*)&p[tmp_bus_map->meta_buffer_offset_soff];
    acomm_bus[busid].comm_buffer_size = (acomm_uint32*)&p[tmp_bus_map->meta_buffer_size_soff];
    acomm_bus[busid].comm_buffer_elmsize = (acomm_uint32*)&p[tmp_bus_map->meta_buffer_elmsize_soff];
    acomm_bus[busid].comm_buffer_earraysize = (acomm_uint32*)&p[tmp_bus_map->meta_buffer_earraysize_soff];
    acomm_bus[busid].comm_buffer_type = (acomm_uint32*)&p[tmp_bus_map->meta_buffer_type_soff];
    acomm_bus[busid].comm_buffer = (acomm_uint8*)&p[tmp_bus_map->data_data_soff];

    return;
}

void acomm_generated_code_init(void)
{
    acomm_uint32 i;

    for (i = 0; i < ATHRILL_COMM_CONFIG_BUS_NUM; i++) {
        acomm_generated_code_init_common(i);
    }
    return;
}

acomm_bus_metadata_type *acomm_open(char *path)
{
    int err;
    struct stat statbuf;
    acomm_bus_metadata_type *tmp_bus_map;
    int fd;

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
    tmp_bus_map = (acomm_bus_metadata_type*)mmap(NULL, statbuf.st_size, (PROT_READ|PROT_WRITE), MAP_SHARED, fd, 0);
    if (tmp_bus_map == NULL) {
        fprintf(stderr, "Error: can not mmap file %s\n", path);
        return NULL;
    }
    bus_fd[tmp_bus_map->meta_busid] = fd;
    bus_map[tmp_bus_map->meta_busid] = tmp_bus_map;
	pthread_mutex_init(&acomm_mutex[tmp_bus_map->meta_busid], NULL);

    acomm_generated_code_init_common(tmp_bus_map->meta_busid);

    return tmp_bus_map;
}

void acomm_close(acomm_bus_metadata_type *p)
{
    int fd;
    if (p == NULL) {
        return;
    }
    if (bus_map[p->meta_busid] != p) {
        return;
    }
    fd = bus_fd[p->meta_busid];
    if (fd >= 0) {
        close(fd);
        bus_fd[p->meta_busid] = -1;
    }
    bus_map[p->meta_busid] = NULL;
    return;
}

void acomm_lock(acomm_busid busid)
{
    int fd;
    if (bus_map[busid] == NULL) {
        return;
    }
    fd = bus_fd[busid];
    if (fd >= 0) {
        pthread_mutex_lock(&acomm_mutex[busid]);
        flock(fd, LOCK_EX);
    }
    return;
}

void acomm_unlock(acomm_busid busid)
{
    int fd;
    if (bus_map[busid] == NULL) {
        return;
    }
    fd = bus_fd[busid];
    if (fd >= 0) {
        flock(fd, LOCK_UN);
        pthread_mutex_unlock(&acomm_mutex[busid]);
    }
    return;
}
