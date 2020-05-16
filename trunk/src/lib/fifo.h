#ifndef _FIFO_H_
#define _FIFO_H_

#include "std_types.h"
#include "std_errno.h"

typedef struct {
	uint32 count;
	uint32 write_off;
	uint32 read_off;
	uint32	buffer_size;
	uint8  *bufferp;
} FifoType;


extern FifoType *fifo_create(uint32 buffer_size);
extern Std_ReturnType fifo_put_char(FifoType *fifop, uint8 data);
extern Std_ReturnType fifo_get_char(FifoType *fifop, uint8 *data);

extern Std_ReturnType fifo_put_data(FifoType *fifop, uint8 *datap, uint32 datalen, uint32 *rlen);
extern Std_ReturnType fifo_get_data(FifoType *fifop, uint8 *datap, uint32 datalen, uint32 *rlen);

#endif /* _FIFO_H_ */
