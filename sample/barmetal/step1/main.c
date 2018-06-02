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

int global_value;
int *global_value_pointer;
int main(void)
{
	global_value_pointer = &global_value;

	*global_value_pointer = 999;

	while (1) {
		;
	}
}
