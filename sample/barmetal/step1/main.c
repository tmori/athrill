#include "test_serial.h"

unsigned char stack_data[1024];

extern void test_suite(void);

unsigned int test_data_uint32;

void test_print(const char *str)
{
	int i;
	for (i = 0; str[i] != '\0'; i++) {
		*(SERIAL_OUT_ADDR) = str[i];
	}
	*(SERIAL_OUT_ADDR) = '\n';
}

int main(void)
{
	test_suite();

	while (1) {
		;
	}
}
