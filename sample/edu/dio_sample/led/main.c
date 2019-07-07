#include "serial.h"
#include "reg.h"
#include "section.h"
#include "interrupt.h"
#include <string.h>
#include "timer.h"
#include "v850_ins.h"
#include "digital.h"

unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

extern void sample_program(void);
/*
 * 100msec timer
 */
static void timer_interrupt_handler(void)
{
	sample_program();
	return;
}

int main(void)
{
	timer_init(timer_interrupt_handler);
	timer_start(50000);

    while (TRUE) {
        do_idle();
    }
    return 0;
}


void bss_clear(void)
{
	unsigned char *p = &_bss_start;
	unsigned char *e = &_bss_end;
	for (;p < e; p++) {
		*p = 0;
	}
	return;
}

void data_init(void)
{
	unsigned char *p_rom = &_idata_start;
	unsigned char *e_rom = &_idata_end;
	unsigned char *p_ram = &_data_start;

	for (;p_rom < e_rom; p_ram++, p_rom++) {
		*p_ram = *p_rom;
	}
}
