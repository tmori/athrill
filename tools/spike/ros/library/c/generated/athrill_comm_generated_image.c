#include "athrill_comm_image.h"
#include "athrill_comm_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "athrill_comm_generated_config.h"

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
     * bus1
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
    /*****************************
    * ELM: CANID_0x100
    *****************************/
    work_offp[0]  = mp->data_data_soff;
    work_sizep[0] = ACOMM_BUS1_ELM_0_SIZE;
    /*****************************
    * ELM: CANID_0x200
    *****************************/
    work_offp[1]  = mp->data_data_soff;
    work_sizep[1] = ACOMM_BUS1_ELM_1_SIZE;
    /*****************************
    * ELM: CANID_0x201
    *****************************/
    work_offp[2]  = mp->data_data_soff;
    work_sizep[2] = ACOMM_BUS1_ELM_2_SIZE;

    /*****************************
    * data elm: CANID_0x100
    *****************************/
    qp = (acomm_queue_type*)&acomm_bus1_mapbuffer[work_offp[0]];
    qp->len = 0U;
    qp->maxlen = ACOMM_BUS1_ELM_0_QUEUE_LEN;
    qp->roff = 0U;
    qp->woff = 0U;
    qp->elmsize = ACOMM_BUS1_ELM_0_SIZE;
    /*****************************
    * data elm: CANID_0x200
    *****************************/
    qp = (acomm_queue_type*)&acomm_bus1_mapbuffer[work_offp[1]];
    qp->len = 0U;
    qp->maxlen = ACOMM_BUS1_ELM_1_QUEUE_LEN;
    qp->roff = 0U;
    qp->woff = 0U;
    qp->elmsize = ACOMM_BUS1_ELM_1_SIZE;
    /*****************************
    * data elm: CANID_0x201
    *****************************/
    arrayp = &acomm_bus1_mapbuffer[work_offp[2]];
    *((acomm_uint8 *)&arrayp[0]) = 0;
    *((acomm_uint8 *)&arrayp[1]) = 2;
    *((acomm_uint8 *)&arrayp[2]) = 0;
    *((acomm_uint8 *)&arrayp[3]) = 4;
    *((acomm_uint8 *)&arrayp[4]) = 0;
    *((acomm_uint8 *)&arrayp[5]) = 0;
    *((acomm_uint8 *)&arrayp[6]) = 8;
    *((acomm_uint8 *)&arrayp[7]) = 0;
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
    mp->meta_buffer_offset_size = ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32);
    mp->meta_buffer_size_soff =  mp->meta_buffer_offset_soff + mp->meta_buffer_offset_size;
    mp->meta_buffer_size_size =  ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32);
    mp->data_data_soff =  mp->meta_buffer_size_soff + mp->meta_buffer_size_size;
    mp->data_data_size =  ACOMM_BUS2_DATA_SIZE;

    /*
     * data region
     */
    /* offset */
    /* size */
    work_offp = (acomm_uint32*)&acomm_bus2_mapbuffer[mp->meta_buffer_offset_soff];
    work_sizep = (acomm_uint32*)&acomm_bus2_mapbuffer[mp->meta_buffer_size_soff];
    /*****************************
    * ELM: CANID_0x101
    *****************************/
    work_offp[0]  = mp->data_data_soff;
    work_sizep[0] = ACOMM_BUS2_ELM_0_SIZE;
    /*****************************
    * ELM: CANID_0x202
    *****************************/
    work_offp[1]  = mp->data_data_soff;
    work_sizep[1] = ACOMM_BUS2_ELM_1_SIZE;
    /*****************************
    * ELM: CANID_0x208
    *****************************/
    work_offp[2]  = mp->data_data_soff;
    work_sizep[2] = ACOMM_BUS2_ELM_2_SIZE;
    /*****************************
    * ELM: CANID_0x209
    *****************************/
    work_offp[3]  = mp->data_data_soff;
    work_sizep[3] = ACOMM_BUS2_ELM_3_SIZE;

    /*****************************
    * data elm: CANID_0x101
    *****************************/
    qp = (acomm_queue_type*)&acomm_bus2_mapbuffer[work_offp[0]];
    qp->len = 0U;
    qp->maxlen = ACOMM_BUS2_ELM_0_QUEUE_LEN;
    qp->roff = 0U;
    qp->woff = 0U;
    qp->elmsize = ACOMM_BUS2_ELM_0_SIZE;
    /*****************************
    * data elm: CANID_0x202
    *****************************/
    qp = (acomm_queue_type*)&acomm_bus2_mapbuffer[work_offp[1]];
    qp->len = 0U;
    qp->maxlen = ACOMM_BUS2_ELM_1_QUEUE_LEN;
    qp->roff = 0U;
    qp->woff = 0U;
    qp->elmsize = ACOMM_BUS2_ELM_1_SIZE;
    /*****************************
    * data elm: CANID_0x208
    *****************************/
    arrayp = &acomm_bus2_mapbuffer[work_offp[2]];
    *((acomm_uint8 *)&arrayp[0]) = 0;
    *((acomm_uint8 *)&arrayp[1]) = 2;
    *((acomm_uint8 *)&arrayp[2]) = 0;
    *((acomm_uint8 *)&arrayp[3]) = 4;
    *((acomm_uint8 *)&arrayp[4]) = 0;
    *((acomm_uint8 *)&arrayp[5]) = 0;
    *((acomm_uint8 *)&arrayp[6]) = 8;
    *((acomm_uint8 *)&arrayp[7]) = 0;
    /*****************************
    * data elm: CANID_0x209
    *****************************/
    arrayp = &acomm_bus2_mapbuffer[work_offp[3]];
    *((acomm_uint32 *)&arrayp[0U]) = 0;
    return ACOMM_E_OK;
}
