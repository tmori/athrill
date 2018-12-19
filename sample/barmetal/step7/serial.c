#include "interrupt_table.h"

#define UD1TX		((volatile char*)0xFFFFFA17)
#define UD1RX		((volatile char*)0xFFFFFA16)
#define UD1STR      ((volatile unsigned char*)0xFFFFFA14)

#define SERIAL_DATA_CACHE_SIZE 1024
typedef struct {
    int count;
    int wcount;
    int rcount;
    char buffer[SERIAL_DATA_CACHE_SIZE];
} SerialBufferType;

static SerialBufferType serial_buffer;

static void serial_interrupt_handler(void)
{
	char dat;
	char str;

	dat = *(UD1RX); /* 受信データを読み込み */

    str = *(UD1STR);
	str &= ~0x10;
    *(UD1STR) = str;

    if (serial_buffer.count >= SERIAL_DATA_CACHE_SIZE) {
        return;
    }
    serial_buffer.buffer[serial_buffer.wcount] = dat;
    serial_buffer.wcount++;
    serial_buffer.count++;
    if (serial_buffer.wcount >= SERIAL_DATA_CACHE_SIZE) {
        serial_buffer.wcount = 0;
    }
	return;
}

void serial_init(void)
{
    register_interrupt_handler(39, serial_interrupt_handler);
    return;
}
int serial_get(char *cp)
{
    if (serial_buffer.count == 0) {
        return 0;
    }
    *cp = serial_buffer.buffer[serial_buffer.rcount];

    serial_buffer.rcount++;
    serial_buffer.count--;
    if (serial_buffer.rcount >= SERIAL_DATA_CACHE_SIZE) {
        serial_buffer.rcount = 0;
    }
    return 1;
}

void serial_put(char c)
{
    (*UD1TX) = c;
    return;
}
