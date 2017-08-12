#ifndef _FIFO_H_
#define _FIFO_H_

typedef unsigned int uint32;
typedef unsigned char uint8;

#define FIFO_DATA_MAX	8
typedef struct {
	uint32 canid;
	uint32  len;
	uint8  data[FIFO_DATA_MAX];
} FifoDataType;

#endif /* _FIFO_H_ */
