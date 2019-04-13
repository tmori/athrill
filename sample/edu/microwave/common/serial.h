#ifndef _TEST_SERIAL_H_
#define _TEST_SERIAL_H_

#define SERIAL_OUT_ADDR		((volatile unsigned char*)0xFFFFFA07)

static inline void test_print(const char *str)
{
	int i;
	for (i = 0; str[i] != '\0'; i++) {
		*(SERIAL_OUT_ADDR) = str[i];
	}
}

static char line_buffer[8];
static const int dec_order[5] = {
	1,
	10,
	100,
	1000,
	10000,
};

static inline char *get_lineno(int lineno) {
	int i;
	int k = 0;
	int div;
	int next_val = lineno;

	for (k = 0; k < 8; k++) {
		line_buffer[k] = '\0';
	}
	k = 0;
	for (i = 4; i >= 0; i--) {
		div = next_val / dec_order[i];
		if ((k > 0) || (div > 0)) {
			line_buffer[k] = div + '0';
			k++;
		}
		next_val -= (div * dec_order[i]);
	}

	return line_buffer;
}

static inline void test_print_line(const char *str, int lineno)
{
	test_print(str);
	test_print(get_lineno(lineno));
	*(SERIAL_OUT_ADDR) = '\n';
}

#define printf test_print

static inline void test_print_one(const char *ch)
{
	*(SERIAL_OUT_ADDR) = *ch;
}
#define UINT_C(arg)		arg
/*
 * UARTDn送信データ・レジスタ（ UDnTX）
 */
#define UDnTX_BASE              UINT_C(0xFFFFFA07)
#define UDnTX(CH)               (UDnTX_BASE + ((CH) * 16U))

/*
 * UARTDn受信データ・レジスタ（ UDnRX）
 */
#define UDnRX_BASE              UINT_C(0xFFFFFA06)
#define UDnRX(CH)               (UDnRX_BASE + ((CH) * 16U))

static inline void athrill_fputs(const char *p)
{
	const char *tmp = p;
	volatile char *outreg = (char*)UDnTX(1);

	while ((*tmp) != '\0') {
		(*outreg) = (*tmp);
		tmp++;
	}
	*outreg = '\n';
	return;
}
static inline void athrill_fputi(int integer)
{
	athrill_fputs(get_lineno(integer));
	return;
}
#endif /* _TEST_SERIAL_H_ */
