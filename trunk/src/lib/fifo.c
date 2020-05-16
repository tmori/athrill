#include "fifo.h"
#include <stdlib.h>

FifoType *fifo_create(uint32 buffer_size)
{
	FifoType *fifop = malloc(sizeof(FifoType));
	if (fifop == NULL) {
		return NULL;
	}
	fifop->bufferp = malloc(buffer_size);
	if (fifop->bufferp == NULL) {
		free(fifop);
		return NULL;
	}
	fifop->buffer_size = buffer_size;
	fifop->count = 0;
	fifop->read_off = 0;
	fifop->write_off = 0;
	return fifop;
}
Std_ReturnType fifo_put_char(FifoType *fifop, uint8 data)
{
	if (fifop->count >= fifop->buffer_size) {
		return STD_E_LIMIT;
	}
	fifop->bufferp[fifop->write_off] = data;

	fifop->count++;
	fifop->write_off++;
	if (fifop->write_off >= fifop->buffer_size) {
		fifop->write_off = 0U;
	}
	return STD_E_OK;
}
Std_ReturnType fifo_get_char(FifoType *fifop, uint8 *data)
{
	if (fifop->count == 0U) {
		return STD_E_NOENT;
	}
	*data = fifop->bufferp[fifop->read_off];

	fifop->count--;
	fifop->read_off++;
	if (fifop->read_off >= fifop->buffer_size) {
		fifop->read_off = 0U;
	}
	return STD_E_OK;
}

Std_ReturnType fifo_put_data(FifoType *fifop, uint8 *datap, uint32 datalen, uint32 *rlen)
{
	uint32 i;
	Std_ReturnType ret;
	*rlen = 0;
	for (i = 0; i < datalen; i++) {
		ret = fifo_put_char(fifop, datap[i]);
		if (ret != STD_E_OK) {
			break;
		}
		(*rlen)++;
	}
	return STD_E_OK;
}
Std_ReturnType fifo_get_data(FifoType *fifop, uint8 *datap, uint32 datalen, uint32 *rlen)
{
	uint32 i;
	Std_ReturnType ret;
	*rlen = 0;
	for (i = 0; i < datalen; i++) {
		ret = fifo_get_char(fifop, &datap[i]);
		if (ret != STD_E_OK) {
			break;
		}
		(*rlen)++;
	}
	return STD_E_OK;
}
