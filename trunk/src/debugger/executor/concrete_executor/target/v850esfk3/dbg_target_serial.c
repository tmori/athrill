#include "concrete_executor/target/dbg_target_serial.h"
#include <stdio.h>

#define DBG_SERIAL_CHANNEL_NUM	4U
#define DBG_SERIAL_BUFFER_SIZE	1024U

typedef struct {
	uint32 count;
	uint32 write_off;
	uint32 read_off;
	uint8  buffer[DBG_SERIAL_BUFFER_SIZE];
} DbgSerialFifoType;

DbgSerialFifoType dbg_serial_fifo[DBG_SERIAL_CHANNEL_NUM];

Std_ReturnType dbg_serial_in(uint8 channel, uint8 data)
{
	DbgSerialFifoType *fifo;
	if (channel >= DBG_SERIAL_CHANNEL_NUM) {
		return STD_E_INVALID;
	}

	fifo = &dbg_serial_fifo[channel];
	if (fifo->count >= DBG_SERIAL_BUFFER_SIZE) {
		return STD_E_LIMIT;
	}
	fifo->buffer[fifo->write_off] = data;

	fifo->count++;
	fifo->write_off++;
	if (fifo->write_off >= DBG_SERIAL_BUFFER_SIZE) {
		fifo->write_off = 0U;
	}
	return STD_E_OK;
}


bool dbg_serial_getchar(uint8 channel, uint8 *data)
{
	DbgSerialFifoType *fifo;
	if (channel >= DBG_SERIAL_CHANNEL_NUM) {
		return FALSE;
	}

	fifo = &dbg_serial_fifo[channel];
	if (fifo->count == 0U) {
		return FALSE;
	}
	*data = fifo->buffer[fifo->read_off];

	fifo->count--;
	fifo->read_off++;
	if (fifo->read_off >= DBG_SERIAL_BUFFER_SIZE) {
		fifo->read_off = 0U;
	}
	return TRUE;
}

bool dbg_serial_putchar(uint8 channel, uint8 data)
{
	printf("%c", data);
	fflush(stdout);
	return TRUE;
}
