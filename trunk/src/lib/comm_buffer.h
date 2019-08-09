#ifndef _COMM_BUFFER_H_
#define _COMM_BUFFER_H_

#include "std_types.h"
#include "std_errno.h"

typedef struct {
	uint32	max_size;
	uint32	count;
	uint32	rx_off;
	uint32	tx_off;
	char*	data;
} CommFifoBufferType;
extern Std_ReturnType comm_fifo_buffer_create(uint32 size, CommFifoBufferType *fifop);
extern Std_ReturnType comm_fifo_buffer_add(CommFifoBufferType *fifop, const char* datap, uint32 datalen, uint32 *res);
extern Std_ReturnType comm_fifo_buffer_get(CommFifoBufferType *fifop, char* datap, uint32 datalen, uint32 *res);
extern void comm_fifo_buffer_close(CommFifoBufferType *fifop);
extern void comm_fifo_buffer_destroy(CommFifoBufferType *fifop);

typedef struct {
	uint32	max_size;
	char*	data;
} CommBufferType;
extern Std_ReturnType comm_buffer_create(uint32 size, CommBufferType *bufferp);
extern void comm_buffer_destroy(CommBufferType *bufferp);


#endif /* _COMM_BUFFER_H_ */
