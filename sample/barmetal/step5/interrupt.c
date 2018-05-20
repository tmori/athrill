#include "interrupt.h"
#include "v850_ins.h"

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

void x_enable_int(int intno)
{
	uint32 intreg_addr = INTREG_ADDRESS(intno);
	
	/* 6bit目をクリア */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) & ~(0x01U << 6));
	
	return;
}
void x_clear_int(int intno)
{
	uint32 intreg_addr = INTREG_ADDRESS(intno);
	
	
	/* 7bit目をクリア */
	sil_wrb_mem((void *)intreg_addr , 
		sil_reb_mem((void *)intreg_addr) & ~(0x01U << 7));
}

void timer_interrupt_handler(void)
{
	return;
}

void interrupt_handler(int intrno)
{
	x_clear_int(intrno);
	enable_int_all();
	if (intrno == 22) {
		timer_interrupt_handler();
	}
	disable_int_all();
	return;
}
