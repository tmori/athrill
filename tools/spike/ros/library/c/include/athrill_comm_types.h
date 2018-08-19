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
typedef unsigned char       acomm_bool;

/*
 * ReturnType
 */
typedef unsigned int        acomm_rtype;

/*
 * QueueType
 */
typedef struct {
    acomm_uint32    maxlen;
    acomm_uint32    elmsize;
    acomm_uint32    len;
    acomm_uint32    woff;
    acomm_uint32    roff;
    acomm_uint8     elements[4];
} acomm_queue_type;
#define ACOMM_QUEUE_ELEM_BUFFER_OFF(entry, off)    ((off) * (entry)->elmsize)

#endif /* _ATHRILL_COMM_TYPES_H_ */