#include "athrill_comm_config.h"
#include "athrill_comm.h"
#include <string.h>

acomm_rtype athrill_comm_write(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size)
{
    acomm_bus_type *bus;
    acomm_uint32 off;

    if (data == NULL_PTR) {
        return ACOMM_E_INVALID;
    }
    if (busid >= ATHRILL_COMM_CONFIG_BUS_NUM) {
        return ACOMM_E_INVALID;
    }
    bus = &acomm_bus[busid];
    if (elmid >= bus->num) {
        return ACOMM_E_INVALID;
    }
    else if (size != bus->comm_buffer_size[elmid]) {
        return ACOMM_E_INVALID;
    }
    off = bus->comm_buffer_offset[elmid];

    /* TODO ex */
    memcpy(&bus->comm_buffer[off], data, size);
    /* TODO ex */

    return ACOMM_E_OK;
}

acomm_rtype athrill_comm_read(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 *size)
{
    acomm_bus_type *bus;
    acomm_uint32 off;

    if (data == NULL_PTR) {
        return ACOMM_E_INVALID;
    }
    if (busid >= ATHRILL_COMM_CONFIG_BUS_NUM) {
        return ACOMM_E_INVALID;
    }
    bus = &acomm_bus[busid];
    if (elmid >= bus->num) {
        return ACOMM_E_INVALID;
    }
    off = bus->comm_buffer_offset[elmid];
    *size = bus->comm_buffer_size[elmid];

    /* TODO ex */
    memcpy(data, &bus->comm_buffer[off], *size);
    /* TODO ex */

    return ACOMM_E_OK;
}
