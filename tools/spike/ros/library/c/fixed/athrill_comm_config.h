#ifndef _ATHRILL_COMM_CONFIG_H_
#define _ATHRILL_COMM_CONFIG_H_

#include "athrill_comm_types.h"
#include "athrill_comm_generated_config.h"

#define ACOMM_BUS_META_VERSION      0x100
#define ACOMM_BUS_META_MAGICNO      0xBEAFDEAD

#define AcommDataType_Primitive     0U
#define AcommDataType_Queue         1U

typedef struct {
    acomm_uint32            meta_version;
    acomm_uint32            meta_magicno;
    acomm_uint32            meta_busid;
    acomm_uint32            meta_buffer_offset_soff;
    acomm_uint32            meta_buffer_offset_size;
    acomm_uint32            meta_buffer_size_soff;
    acomm_uint32            meta_buffer_size_size;
    acomm_uint32            meta_buffer_type_soff;
    acomm_uint32            meta_buffer_type_size;
    acomm_uint32            data_data_soff;
    acomm_uint32            data_data_size;
} acomm_bus_metadata_type;
#define ACOMM_BUS_METADATA_SIZE     sizeof(acomm_bus_metadata_type)

typedef struct {
    acomm_uint32            num;
    acomm_uint32            *comm_buffer_offset;
    acomm_uint32            *comm_buffer_size;
    acomm_uint32            *comm_buffer_type;
    acomm_uint8             *comm_buffer;
} acomm_bus_type;

extern acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];
extern void acomm_generated_code_init(void);

#endif /* _ATHRILL_COMM_CONFIG_H_ */