#include "test_serial.h"
#include "test_reg.h"
#include "section.h"
#include <string.h>

unsigned char stack_data[STACK_SIZE] __attribute__ ((section("bss_noclr")));

static int global_value;
static int *global_value_pointer = &global_value;

int main(void)
{
	*global_value_pointer = 999;

	printf("Hello World!!\n");
	while (1) {
		;
	}
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
