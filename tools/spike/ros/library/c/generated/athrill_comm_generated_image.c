#include "athrill_comm_image.h"
#include "athrill_comm_config.h"
#include <stdio.h>
#include <stdlib.h>

static acomm_uint8 acomm_bus1_mapbuffer[ACOMM_BUS1_BUFFER_SIZE];
static acomm_uint8 acomm_bus2_mapbuffer[ACOMM_BUS2_BUFFER_SIZE];


acomm_rtype athrill_comm_make_image(void)
{
    acomm_bus_metadata_type *mp;
    acomm_uint32 *work_offp;
    acomm_uint32 *work_sizep;
    acomm_queue_type *qp;
    acomm_uint8 *arrayp;

    /********************************************
     * bus2
     ********************************************/
    /*
     * meta data region
     */
    mp = (acomm_bus_metadata_type*)acomm_bus1_mapbuffer;
    mp->meta_version = ACOMM_BUS_META_VERSION;
    mp->meta_magicno = ACOMM_BUS_META_MAGICNO;
    mp->meta_buffer_offset_soff = ACOMM_BUS_METADATA_SIZE;
    mp->meta_buffer_offset_size = ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32);
    mp->meta_buffer_size_soff =  mp->meta_buffer_offset_soff + mp->meta_buffer_offset_size;
    mp->meta_buffer_size_size =  ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32);
    mp->data_data_soff =  mp->meta_buffer_size_soff + mp->meta_buffer_size_size;
    mp->data_data_size =  ACOMM_BUS1_DATA_SIZE;
    /*
     * data region
     */
    /* offset */
    /* size */
    work_offp = (acomm_uint32*)&acomm_bus1_mapbuffer[mp->meta_buffer_offset_soff];
    work_sizep = (acomm_uint32*)&acomm_bus1_mapbuffer[mp->meta_buffer_size_soff];

    work_offp[0]  = mp->data_data_soff;
    work_sizep[0] = ACOMM_BUS1_ELM_0_SIZE;

    work_offp[1]  = work_offp[0] + work_sizep[0];
    work_sizep[1] = ACOMM_BUS1_ELM_1_SIZE;

    work_offp[2]  = work_offp[1] + work_sizep[1];
    work_sizep[2] = ACOMM_BUS1_ELM_2_SIZE;

    /* data */
    /* elm1 */
    qp = (acomm_queue_type*)&acomm_bus1_mapbuffer[work_offp[0]];
    qp->len = 0U;
    qp->maxlen = ACOMM_BUS1_ELM_0_QUEUE_LEN;
    qp->roff = 0U;
    qp->woff = 0U;
    qp->elmsize = ACOMM_BUS1_ELM_0_SIZE;

    /* elm2 */
    qp = (acomm_queue_type*)&acomm_bus1_mapbuffer[work_offp[1]];
    qp->len = 0U;
    qp->maxlen = ACOMM_BUS1_ELM_1_QUEUE_LEN;
    qp->roff = 0U;
    qp->woff = 0U;
    qp->elmsize = ACOMM_BUS1_ELM_1_SIZE;

    /* elm3 */
    arrayp = &acomm_bus1_mapbuffer[work_offp[2]];
    arrayp[0] = 0U;
    arrayp[1] = 0U;
    arrayp[2] = 0U;
    arrayp[3] = 0U;
    arrayp[4] = 0U;
    arrayp[5] = 0U;
    arrayp[6] = 0U;
    arrayp[7] = 0U;

    return ACOMM_E_OK;
}
