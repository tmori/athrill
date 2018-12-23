#include "test_serial.h"
#include "test_reg.h"
#include <string.h>
#include "athrill_syscall.h"
#include <math.h>
#include <stdlib.h>


unsigned char stack_data[STACK_SIZE] __attribute__ ((section(".bss_noclr")));

sys_addr athrill_device_func_call __attribute__ ((section(".athrill_device_section")));



int main(void)
{
	void *addr[4];
	char *p;

	addr[0] = malloc(31);
	addr[1] = malloc(32);
	addr[2] = malloc(33);

	free(addr[0]);

	addr[3] = malloc(11);

	p = (char*)addr[3];

	p[0] = 't';
	p[1] = 'm';
	p[2] = 'o';
	p[3] = 'r';
	p[4] = 'i';
	p[5] = '\0';
	while (1) {
		;
	}
}
