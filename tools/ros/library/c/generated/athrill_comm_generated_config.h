#ifndef _ATHRILL_COMM_GENERATED_CONFIG_H_
#define _ATHRILL_COMM_GENERATED_CONFIG_H_

#define ATHRILL_COMM_CONFIG_BUS_NUM     2U

/*****************************
 * BUS: bus1
 *****************************/
#define ACOMM_BUS1_ELEMENT_NUM  2U
/*****************************
 * ELM: CANID_0x100
 *****************************/
#define ACOMM_BUS1_ELM_0_ARRAY_SIZE    ((8U))
#define ACOMM_BUS1_ELM_0_TYPE_SIZE    ((1U) * (8U))
#define ACOMM_BUS1_ELM_0_TYPE    AcommDataType_Queue
#define ACOMM_BUS1_ELM_0_QUEUE_LEN    (100U)
#define ACOMM_BUS1_ELM_0_RAW_SIZE     ( ACOMM_BUS1_ELM_0_QUEUE_LEN * ACOMM_BUS1_ELM_0_TYPE_SIZE )
#define ACOMM_BUS1_ELM_0_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS1_ELM_0_QUEUE_LEN * ACOMM_BUS1_ELM_0_TYPE_SIZE ) \
    )
/*****************************
 * ELM: CANID_0x101
 *****************************/
#define ACOMM_BUS1_ELM_1_ARRAY_SIZE    ((8U))
#define ACOMM_BUS1_ELM_1_TYPE_SIZE    ((1U) * (8U))
#define ACOMM_BUS1_ELM_1_TYPE    AcommDataType_Queue
#define ACOMM_BUS1_ELM_1_QUEUE_LEN    (100U)
#define ACOMM_BUS1_ELM_1_RAW_SIZE     ( ACOMM_BUS1_ELM_1_QUEUE_LEN * ACOMM_BUS1_ELM_1_TYPE_SIZE )
#define ACOMM_BUS1_ELM_1_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS1_ELM_1_QUEUE_LEN * ACOMM_BUS1_ELM_1_TYPE_SIZE ) \
    )

/*
 * BUS1 data size
 */
#define ACOMM_BUS1_DATA_SIZE    (                                   \
    ACOMM_BUS1_ELM_0_SIZE +        \
    ACOMM_BUS1_ELM_1_SIZE          \
)
/*
 * BUS1 buffer size
 */
#define ACOMM_BUS1_BUFFER_SIZE  (                                   \
        ACOMM_BUS_METADATA_SIZE                         +               \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        ACOMM_BUS1_DATA_SIZE                                           \
    )

/*****************************
 * BUS: bus2
 *****************************/
#define ACOMM_BUS2_ELEMENT_NUM  2U
/*****************************
 * ELM: CANID_0x200
 *****************************/
#define ACOMM_BUS2_ELM_0_ARRAY_SIZE    ((8U))
#define ACOMM_BUS2_ELM_0_TYPE_SIZE    ((1U) * (8U))
#define ACOMM_BUS2_ELM_0_TYPE    AcommDataType_Queue
#define ACOMM_BUS2_ELM_0_QUEUE_LEN    (100U)
#define ACOMM_BUS2_ELM_0_RAW_SIZE     ( ACOMM_BUS2_ELM_0_QUEUE_LEN * ACOMM_BUS2_ELM_0_TYPE_SIZE )
#define ACOMM_BUS2_ELM_0_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS2_ELM_0_QUEUE_LEN * ACOMM_BUS2_ELM_0_TYPE_SIZE ) \
    )
/*****************************
 * ELM: CANID_0x201
 *****************************/
#define ACOMM_BUS2_ELM_1_ARRAY_SIZE    ((8U))
#define ACOMM_BUS2_ELM_1_TYPE_SIZE    ((1U) * (8U))
#define ACOMM_BUS2_ELM_1_TYPE    AcommDataType_Queue
#define ACOMM_BUS2_ELM_1_QUEUE_LEN    (100U)
#define ACOMM_BUS2_ELM_1_RAW_SIZE     ( ACOMM_BUS2_ELM_1_QUEUE_LEN * ACOMM_BUS2_ELM_1_TYPE_SIZE )
#define ACOMM_BUS2_ELM_1_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS2_ELM_1_QUEUE_LEN * ACOMM_BUS2_ELM_1_TYPE_SIZE ) \
    )

/*
 * BUS2 data size
 */
#define ACOMM_BUS2_DATA_SIZE    (                                   \
    ACOMM_BUS2_ELM_0_SIZE +        \
    ACOMM_BUS2_ELM_1_SIZE          \
)
/*
 * BUS2 buffer size
 */
#define ACOMM_BUS2_BUFFER_SIZE  (                                   \
        ACOMM_BUS_METADATA_SIZE                         +               \
        (ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS2_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        ACOMM_BUS2_DATA_SIZE                                           \
    )

#endif