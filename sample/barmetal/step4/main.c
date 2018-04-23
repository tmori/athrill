#include "test_serial.h"
#include "test_reg.h"
#include "section.h"
#include "list_test.h"
#include "list.h"
#include <string.h>

unsigned char stack_data[STACK_SIZE] __attribute__ ((section("bss_noclr")));


int main(void)
{
	list_test_ERR_1();
	list_test_ERR_2();
	list_test_OK_1();
	list_test_OK_2();
	list_test_OK_3();
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
