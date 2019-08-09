#include "comm_buffer.h"
#include <stdlib.h>

Std_ReturnType comm_fifo_buffer_create(uint32 size, CommFifoBufferType *fifop)
{
	if (fifop == NULL) {
		return STD_E_INVALID;
	}
	fifop->data = malloc(size);
	if (fifop->data == NULL) {
		return STD_E_LIMIT;
	}
	fifop->max_size = size;
	fifop->count = 0;
	fifop->rx_off = 0;
	fifop->tx_off = 0;
	return STD_E_OK;
}

static Std_ReturnType fifo_buffer_add(CommFifoBufferType *fifop, char data)
{
	if (fifop->count >= fifop->max_size) {
		return STD_E_LIMIT;
	}
	fifop->data[fifop->tx_off] = data;
	fifop->tx_off++;
	if (fifop->tx_off >= fifop->max_size) {
		fifop->tx_off = 0;
	}
	fifop->count++;
	return STD_E_OK;
}

Std_ReturnType comm_fifo_buffer_add(CommFifoBufferType *fifop, const char* datap, uint32 datalen, uint32 *res)
{
	uint32 i;
	Std_ReturnType err;

	if (fifop->count >= fifop->max_size) {
		return STD_E_LIMIT;
	}
	for (i = 0; i < datalen; i++) {
		err = fifo_buffer_add(fifop, datap[i]);
		if (err != STD_E_OK) {
			return err;
		}
		(*res) = (*res) + 1;
	}
	return STD_E_OK;
}
static Std_ReturnType fifo_buffer_get(CommFifoBufferType *fifop, char* datap)
{
	if (fifop->count == 0) {
		return STD_E_NOENT;
	}
	*datap = fifop->data[fifop->rx_off];
	fifop->rx_off++;
	if (fifop->rx_off >= fifop->max_size) {
		fifop->rx_off = 0;
	}
	fifop->count--;
	return STD_E_OK;
}

Std_ReturnType comm_fifo_buffer_get(CommFifoBufferType *fifop, char* datap, uint32 datalen, uint32 *res)
{
	uint32 i;
	Std_ReturnType err;

	if (fifop->count == 0) {
		return STD_E_NOENT;
	}
	for (i = 0; i < datalen; i++) {
		err = fifo_buffer_get(fifop, &datap[i]);
		if (err != STD_E_OK) {
			return err;
		}
		(*res) = (*res) + 1;
	}
	return STD_E_OK;
}

void comm_fifo_buffer_close(CommFifoBufferType *fifop)
{
	if (fifop != NULL) {
		fifop->count = 0;
		fifop->rx_off = 0;
		fifop->tx_off = 0;
	}
	return;
}

void comm_fifo_buffer_destroy(CommFifoBufferType *fifop)
{
	if (fifop != NULL) {
		if (fifop->data != NULL) {
			free(fifop->data);
			fifop->data = NULL;
		}
		fifop->max_size = 0;
		fifop->count = 0;
		fifop->rx_off = 0;
		fifop->tx_off = 0;
	}
	return;
}


Std_ReturnType comm_buffer_create(uint32 size, CommBufferType *bufferp)
{
	if (bufferp == NULL) {
		return STD_E_INVALID;
	}
	bufferp->data = malloc(size);
	if (bufferp->data == NULL) {
		return STD_E_LIMIT;
	}
	bufferp->max_size = size;
	return STD_E_OK;
}
void comm_buffer_destroy(CommBufferType *bufferp)
{
	if (bufferp != NULL) {
		if (bufferp->data != NULL) {
			free(bufferp->data);
			bufferp->data = NULL;
		}
		bufferp->max_size = 0;
	}
	return;
}
