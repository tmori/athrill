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

typedef struct {
    int isArray;
    int size;
    unsigned char *rawdatap;
    union {
        unsigned char uint8_data;
        unsigned short uint16_data;
        unsigned int uint32_data;
        unsigned char uint8_array[8];
    } data;
} parsed_data_type;

static int acomm_write_entry(int index, parsed_data_type *in)
{
    acomm_rtype err;

    if (acomm_bus[bus_map->meta_busid].comm_buffer_type[index] == AcommDataType_Queue) {
        err = athrill_comm_send(bus_map->meta_busid, index, in->rawdatap, acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[index]);
    }
    else {
        err = athrill_comm_write(bus_map->meta_busid, index, in->rawdatap, acomm_bus[bus_map->meta_busid].comm_buffer_elmsize[index]);
    }
    if (err != ACOMM_E_OK) {
        fprintf(stderr, "ERROR:can not write index(%d). err=%d\n", index, err);
    }
    return err;
}

static int parse(char* sizep, char* data, parsed_data_type *out)
{
    char* tp;
    out->size = atoi(sizep);
    if (strstr(data, ",") != NULL) {
        int cnt = 0;
        if (out->size != 8) {
            fprintf(stderr, "ERROR: not supported array size(%d) is invalid\n", out->size);
            return -1;
        }
        tp = strtok(data, ",");
        while( tp != NULL && cnt < 8 ) {
            out->data.uint8_array[cnt++] = atoi(tp);
            tp = strtok(NULL, ",");
            printf("%d\n", out->data.uint8_array[cnt-1]);
        }
        out->isArray = 1;
        out->rawdatap = &out->data.uint8_array[0];
    }
    else {
        out->isArray = 0;
        switch (out->size) {
        case 1:
            out->data.uint8_data = atoi(data);
            out->rawdatap = (unsigned char*)&out->data.uint8_data;
            printf("%d\n", out->data.uint8_data);
            break;
        case 2:
            out->data.uint16_data = atoi(data);
            out->rawdatap = (unsigned char*)&out->data.uint16_data;
            printf("%d\n", out->data.uint16_data);
            break;
        case 4:
            out->data.uint32_data = atoi(data);
            out->rawdatap = (unsigned char*)&out->data.uint32_data;
            printf("%d\n", out->data.uint32_data);
            break;
        default:
            fprintf(stderr, "ERROR: size(%d) is invalid\n", out->size);
            return -1;
        }
    }
    return 0;
}

int main(int argc, const char *argv[])
{
    int i;
	int fd;
	int err;
	struct stat statbuf;
    char *indexp;
    char *sizep;
    char *datap;
    char *path;
    parsed_data_type parsed_data;

    if (argc != 5) {
        fprintf(stderr, "Usage: %s <filepath> <index> <size> <data>\n", argv[0]);
        return 1;
    }
    path = (char*)argv[1];
    indexp = (char*)argv[2];
    sizep = (char*)argv[3];
    datap = (char*)argv[4];
    i = atoi(indexp);

    err = parse(sizep, datap, &parsed_data);
    if (err != 0) {
        fprintf(stderr, "Error: can not parse %s\n", datap);
        return 1;
    }


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

    err = acomm_write_entry(i, &parsed_data);
    if (err != 0) {
        fprintf(stderr, "Error: can not write file %s\n", path);
        return 1;
    }

    close(fd);
    return 0;
}