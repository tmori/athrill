#ifndef _ATHRILL_COMM_CONFIG_H_
#define _ATHRILL_COMM_CONFIG_H_

#include "athrill_comm_types.h"
#include "athrill_comm_generated_config.h"

typedef struct {
    acomm_uint32            num;
    acomm_uint32            *comm_buffer_offset;
    acomm_uint32            *comm_buffer_size;
    acomm_uint8             *comm_buffer;
} acomm_bus_type;

extern acomm_bus_type acomm_bus[ATHRILL_COMM_CONFIG_BUS_NUM];
extern void acomm_generated_code_init(void);

#endif /* _ATHRILL_COMM_CONFIG_H_ */