#include "section.h"


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

typedef void (*constructor)(void);

void ctors_init()
{
#if 0
	unsigned char *p = (void*)&_ctors_start;
	for(;p < &_ctors_end; p += sizeof(constructor))
	{
		void (*func)(void) = (void*)p;
		(*func)();
	}
#endif
}