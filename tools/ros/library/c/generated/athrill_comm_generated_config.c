#include "athrill_comm_config.h"
#include "athrill_comm_generated_config.h"

acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];

static acomm_uint8 acomm_bus1_mapbuffer[ACOMM_BUS1_BUFFER_SIZE] __attribute__ ((section(".mmap_bus1_section")));
static acomm_uint8 acomm_bus2_mapbuffer[ACOMM_BUS2_BUFFER_SIZE] __attribute__ ((section(".mmap_bus2_section")));
static unsigned int athrill_device_func_call __attribute__ ((section(".athrill_device_section")));

void acomm_generated_code_init(void)
{
    acomm_bus_metadata_type *bus_map;
    acomm_uint8 *p;

    bus_map = (acomm_bus_metadata_type*)&acomm_bus1_mapbuffer[0];
    p = (acomm_uint8*)bus_map;
    acomm_bus[0].num = bus_map->meta_entrynum;
    acomm_bus[0].meta = bus_map;
    acomm_bus[0].comm_buffer_offset = (acomm_uint32*)&p[bus_map->meta_buffer_offset_soff];
    acomm_bus[0].comm_buffer_size = (acomm_uint32*)&p[bus_map->meta_buffer_size_soff];
    acomm_bus[0].comm_buffer_elmsize = (acomm_uint32*)&p[bus_map->meta_buffer_elmsize_soff];
    acomm_bus[0].comm_buffer_earraysize = (acomm_uint32*)&p[bus_map->meta_buffer_earraysize_soff];
    acomm_bus[0].comm_buffer_type = (acomm_uint32*)&p[bus_map->meta_buffer_type_soff];
    acomm_bus[0].comm_buffer = (acomm_uint8*)&p[bus_map->data_data_soff];

    bus_map = (acomm_bus_metadata_type*)&acomm_bus2_mapbuffer[1];
    p = (acomm_uint8*)bus_map;
    acomm_bus[1].num = bus_map->meta_entrynum;
    acomm_bus[1].meta = bus_map;
    acomm_bus[1].comm_buffer_offset = (acomm_uint32*)&p[bus_map->meta_buffer_offset_soff];
    acomm_bus[1].comm_buffer_size = (acomm_uint32*)&p[bus_map->meta_buffer_size_soff];
    acomm_bus[1].comm_buffer_elmsize = (acomm_uint32*)&p[bus_map->meta_buffer_elmsize_soff];
    acomm_bus[1].comm_buffer_earraysize = (acomm_uint32*)&p[bus_map->meta_buffer_earraysize_soff];
    acomm_bus[1].comm_buffer_type = (acomm_uint32*)&p[bus_map->meta_buffer_type_soff];
    acomm_bus[1].comm_buffer = (acomm_uint8*)&p[bus_map->data_data_soff];


    return;
}

static void acomm_lock_unlock_common(acomm_busid busid)
{
    switch (busid) {
    case 0:
        athrill_device_func_call = (unsigned int)&acomm_bus1_mapbuffer[0];
        break;
    case 1:
        athrill_device_func_call = (unsigned int)&acomm_bus2_mapbuffer[0];
        break;
    default:
        break;
    }
    return;
}

void acomm_lock(acomm_busid busid)
{
    acomm_lock_unlock_common(busid);
    return;
}

void acomm_unlock(acomm_busid busid)
{
    acomm_lock_unlock_common(busid);
    return;
}
