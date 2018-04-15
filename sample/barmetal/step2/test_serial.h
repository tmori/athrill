#ifndef _TEST_SERIAL_H_
#define _TEST_SERIAL_H_

#define SERIAL_OUT_ADDR		((volatile unsigned char*)0xFFFFFA07)

static inline void test_print(const char *str)
{
	int i;
	for (i = 0; str[i] != '\0'; i++) {
		*(SERIAL_OUT_ADDR) = str[i];
	}
	*(SERIAL_OUT_ADDR) = '\n';
}

#define printf test_print

static inline void test_print_one(const char *ch)
{
	*(SERIAL_OUT_ADDR) = *ch;
}

#endif /* _TEST_SERIAL_H_ */
