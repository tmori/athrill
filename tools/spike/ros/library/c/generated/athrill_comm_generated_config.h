#ifndef _ATHRILL_COMM_GENERATED_CONFIG_H_
#define _ATHRILL_COMM_GENERATED_CONFIG_H_

#define ATHRILL_COMM_CONFIG_BUS_NUM     2U

/*****************************
 * bus1
 *****************************/
#define ACOMM_BUS1_ELEMENT_NUM  3U
#define ACOMM_BUS1_ELM_0_QUEUE_LEN    (1U)
#define ACOMM_BUS1_ELM_0_TYPE_SIZE    (8U)
#define ACOMM_BUS1_ELM_0_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS1_ELM_0_QUEUE_LEN * ACOMM_BUS1_ELM_0_TYPE_SIZE ) \
    )

#define ACOMM_BUS1_ELM_1_QUEUE_LEN    (1U)
#define ACOMM_BUS1_ELM_1_TYPE_SIZE    (8U)
#define ACOMM_BUS1_ELM_1_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS1_ELM_1_QUEUE_LEN * ACOMM_BUS1_ELM_1_TYPE_SIZE ) \
    )

#define ACOMM_BUS1_ELM_2_TYPE_SIZE    (8U)
#define ACOMM_BUS1_ELM_2_SIZE    (                                  \
        ( ACOMM_BUS1_ELM_1_TYPE_SIZE )                              \
    )

#define ACOMM_BUS1_BUFFER_SIZE  (                                   \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        (ACOMM_BUS1_ELEMENT_NUM * sizeof(acomm_uint32)) +               \
        ACOMM_BUS1_ELM_0_SIZE                     +                     \
        ACOMM_BUS1_ELM_1_SIZE                     +                     \
        ACOMM_BUS1_ELM_2_SIZE                                           \
    )

/*****************************
 * bus2
 *****************************/
#define ACOMM_BUS2_ELEMENT_NUM  3U
#define ACOMM_BUS2_ELM_0_QUEUE_LEN    (1U)
#define ACOMM_BUS2_ELM_0_TYPE_SIZE    (8U)
#define ACOMM_BUS2_ELM_0_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS2_ELM_0_QUEUE_LEN * ACOMM_BUS2_ELM_0_TYPE_SIZE ) \
    )

#define ACOMM_BUS2_ELM_1_QUEUE_LEN    (15U)
#define ACOMM_BUS2_ELM_1_TYPE_SIZE    (8U)
#define ACOMM_BUS2_ELM_1_SIZE    (                                  \
        sizeof(acomm_queue_type) +                                  \
        ( ACOMM_BUS2_ELM_1_QUEUE_LEN * ACOMM_BUS2_ELM_1_TYPE_SIZE ) \
    )

#define ACOMM_BUS2_ELM_2_TYPE_SIZE    (8U)
#define ACOMM_BUS2_ELM_2_SIZE    (                                  \
        ( ACOMM_BUS2_ELM_1_TYPE_SIZE )                              \
    )

#define ACOMM_BUS2_BUFFER_SIZE  (   \
        ACOMM_BUS2_ELM_0_SIZE +      \
        ACOMM_BUS2_ELM_1_SIZE +      \
        ACOMM_BUS2_ELM_2_SIZE        \
    )

#endif