#ifndef _ATHRILL_COMM_TYPES_H_
#define _ATHRILL_COMM_TYPES_H_

#ifndef TRUE
#define TRUE    1
#endif

#ifndef FALSE
#define FALSE    1
#endif

#ifndef NULL_PTR
#define NULL_PTR    ((void*)0)
#endif

/*
 * PrimitiveTypes
 */ 
typedef unsigned char       acomm_uint8;
typedef unsigned short      acomm_uint16;
typedef unsigned int        acomm_uint32;
typedef unsigned int        acomm_busid;
typedef unsigned int        acomm_elmid;

/*
 * ReturnType
 */
typedef unsigned int        acomm_rtype;

/*
 * QueueType
 */
typedef struct {
    acomm_uint32    maxlen;
    acomm_uint32    len;
    acomm_uint32    elmsize;
    void            *elements;
} acomm_queue_type;

#endif /* _ATHRILL_COMM_TYPES_H_ */