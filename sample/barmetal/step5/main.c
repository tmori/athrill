#include "test_serial.h"
#include "test_reg.h"
#include "section.h"
#include <string.h>

typedef unsigned char uint8;
typedef unsigned int uint32;
typedef unsigned char bool;
#define TRUE 1
#define FALSE 0

unsigned char stack_data[STACK_SIZE] __attribute__ ((section("bss_noclr")));

static void sil_wrb_mem(void *addr, uint8 data)
{
	(*(uint8*)addr) = data;
	return;
}
static uint8 sil_reb_mem(void *addr)
{
	return (*(uint8*)addr);
}
void default_int_handler(void)
{
	return;
}

/*
 *	割込み制御レジスタの番地を算出するためのマクロ
 *
 *	割込み制御レジスタは割込み番号順に並んでいるため，
 *	ベースアドレスからのオフセットでアドレスを求めることができる．
 */

#define INTREG_BASE				(0xFFFFF110)
#define INTREG_ADDRESS(intno)	(INTREG_BASE + ((intno) * 2U))
static void x_enable_int(int intno)
{
	uint32 intreg_addr = INTREG_ADDRESS(intno);
	
	/* 6bit目をクリア */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) & ~(0x01U << 6));
	
	return;
}
static void x_clear_int(int intno)
{
	uint32 intreg_addr = INTREG_ADDRESS(intno);
	
	
	/* 7bit目をクリア */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) & ~(0x01U << 7));
}

int main(void)
{
	x_enable_int(22);
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

void timer_interrupt_handler(void)
{
	return;
}

void interrupt_handler(int intrno)
{
	x_clear_int(intrno);
	if (intrno == 22) {
		timer_interrupt_handler();
	}
	return;
}

