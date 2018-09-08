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
    off = bus->comm_buffer_offset[elmid] - bus->meta->data_data_soff;

    acomm_lock(busid);
    memcpy(&bus->comm_buffer[off], data, size);
    acomm_unlock(busid);

    return ACOMM_E_OK;
}

acomm_rtype athrill_comm_read(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size)
{
    acomm_rtype err = ACOMM_E_OK;
    acomm_bus_type *bus;
    acomm_uint32 off;

    if (data == NULL_PTR) {
        err = ACOMM_E_INVALID;
        goto done;
    }
    if (busid >= ATHRILL_COMM_CONFIG_BUS_NUM) {
        err = ACOMM_E_INVALID;
        goto done;
    }
    bus = &acomm_bus[busid];
    if (elmid >= bus->num) {
        err = ACOMM_E_INVALID;
        goto done;
    }
    else if (size != bus->comm_buffer_size[elmid]) {
        err = ACOMM_E_INVALID;
        goto done;
    }
    off = bus->comm_buffer_offset[elmid] - bus->meta->data_data_soff;

    acomm_lock(busid);
    memcpy(data, &bus->comm_buffer[off], size);
    acomm_unlock(busid);

done:
    return err;
}

static acomm_rtype athrill_comm_check(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data)
{
    acomm_bus_type *bus;

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
    return ACOMM_E_OK;
}

static acomm_rtype athrill_comm_send_common(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size, acomm_bool forceflag)
{
    acomm_rtype err = ACOMM_E_OK;
    acomm_uint32 boff;
    acomm_bus_type *bus;
    acomm_uint32 off;
    acomm_queue_type *entry;

    acomm_lock(busid);
    err = athrill_comm_check(busid, elmid, data);
    if (err != ACOMM_E_OK) {
        goto done;
    }

    bus = &acomm_bus[busid];
    off = bus->comm_buffer_offset[elmid] - bus->meta->data_data_soff;
    entry = (acomm_queue_type*)&bus->comm_buffer[off];
    if (entry->elmsize != size) {
        err = ACOMM_E_INVALID;
        goto done;
    }
    if (forceflag == FALSE) {
        if (entry->len >= entry->maxlen) {
            err = ACOMM_E_LIMIT;
            goto done;
        }
        boff = ACOMM_QUEUE_ELEM_BUFFER_OFF(entry, entry->woff);
        memcpy(&entry->elements[boff], data, entry->elmsize);
        entry->len++;
        entry->woff++;
        if (entry->woff >= entry->maxlen) {
            entry->woff = 0U;
        }
    }
    else { /* force set on top */
        boff = ACOMM_QUEUE_ELEM_BUFFER_OFF(entry, 0U);
        memcpy(&entry->elements[boff], data, entry->elmsize);
        entry->len = 1U;
        entry->woff = 1U;
        entry->roff = 0U;
    }

done:
    acomm_unlock(busid);
    return err;
}

static acomm_rtype athrill_comm_recv_common(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size, acomm_bool isPeek)
{
    acomm_rtype err = ACOMM_E_OK;
    acomm_bus_type *bus;
    acomm_uint32 boff;
    acomm_uint32 off;
    acomm_queue_type *entry;

    acomm_lock(busid);
    err = athrill_comm_check(busid, elmid, data);
    if (err != ACOMM_E_OK) {
        goto done;
    }

    bus = &acomm_bus[busid];
    off = bus->comm_buffer_offset[elmid] - bus->meta->data_data_soff;
    entry = (acomm_queue_type*)&bus->comm_buffer[off];
    if (entry->elmsize != size) {
        err = ACOMM_E_INVALID;
        goto done;
    }

    if (entry->len == 0U) {
        err = ACOMM_E_NOENT;
        goto done;
    }
    boff = ACOMM_QUEUE_ELEM_BUFFER_OFF(entry, entry->roff);
    memcpy(data, &entry->elements[boff], entry->elmsize);
    if (isPeek == FALSE) {
        entry->len--;
        entry->roff++;
        if (entry->roff >= entry->maxlen) {
            entry->roff = 0U;
        }
    }
done:
    acomm_unlock(busid);
    return err;

}

acomm_rtype athrill_comm_send(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size)
{
    return athrill_comm_send_common(busid, elmid, data, size, FALSE);
}
acomm_rtype athrill_comm_send_force(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size)
{
    return athrill_comm_send_common(busid, elmid, data, size, TRUE);
}

acomm_rtype athrill_comm_recv(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size)
{
    return athrill_comm_recv_common(busid, elmid, data, size, FALSE);
}
acomm_rtype athrill_comm_peek(acomm_busid busid, acomm_elmid elmid, acomm_uint8 *data, acomm_uint32 size)
{
    return athrill_comm_recv_common(busid, elmid, data, size, TRUE);
}
acomm_bool athrill_comm_is_arrived(acomm_busid busid, acomm_elmid elmid)
{
    acomm_bool ret = FALSE;
    acomm_bus_type *bus;
    acomm_uint32 off;
    acomm_queue_type *entry;

    if (busid >= ATHRILL_COMM_CONFIG_BUS_NUM) {
        return FALSE;
    }
    bus = &acomm_bus[busid];
    if (elmid >= bus->num) {
        return FALSE;
    }
    bus = &acomm_bus[busid];
    off = bus->comm_buffer_offset[elmid] - bus->meta->data_data_soff;
    entry = (acomm_queue_type*)&bus->comm_buffer[off];
    if (entry->len > 0) {
        ret = TRUE;
    }
    else {
        ret = FALSE;
    }
    return ret;
}

acomm_rtype athrill_comm_init(void)
{
    acomm_generated_code_init();
    return ACOMM_E_OK;
}
