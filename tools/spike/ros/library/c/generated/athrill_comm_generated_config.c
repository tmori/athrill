#include "athrill_comm_config.h"
#include "athrill_comm_generated_config.h"

acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];

static acomm_uint8 acomm_bus1_mapbuffer[ACOMM_BUS_METADATA_SIZE + ACOMM_BUS1_BUFFER_SIZE] __attribute__ ((section(".mmap_bus1_section")));
static acomm_uint8 acomm_bus2_mapbuffer[ACOMM_BUS_METADATA_SIZE + ACOMM_BUS2_BUFFER_SIZE] __attribute__ ((section(".mmap_bus2_section")));

void acomm_generated_code_init(void)
{
    acomm_bus_metadata_type *mp;

    /*****************************
     * bus1
     *****************************/
    mp = (acomm_bus_metadata_type*)&acomm_bus1_mapbuffer[0];
    acomm_bus[0].num = ACOMM_BUS1_ELEMENT_NUM;
    acomm_bus[0].comm_buffer_offset = (acomm_uint32*)(((acomm_uint32)mp) + (mp->meta_buffer_offset_soff));
    acomm_bus[0].comm_buffer_size = (acomm_uint32*)(((acomm_uint32)mp) + (mp->meta_buffer_size_soff));
    acomm_bus[0].comm_buffer = (acomm_uint8*)(((acomm_uint32)mp) + (mp->data_data_soff));

    /*****************************
     * bus1
     *****************************/
    mp = (acomm_bus_metadata_type*)&acomm_bus2_mapbuffer[0];
    acomm_bus[1].num = ACOMM_BUS2_ELEMENT_NUM;
    acomm_bus[1].comm_buffer_offset = (acomm_uint32*)(((acomm_uint32)mp) + (mp->meta_buffer_offset_soff));
    acomm_bus[1].comm_buffer_size = (acomm_uint32*)(((acomm_uint32)mp) + (mp->meta_buffer_size_soff));
    acomm_bus[1].comm_buffer = (acomm_uint8*)(((acomm_uint32)mp) + (mp->data_data_soff));

    return;
}