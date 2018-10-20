#include "interrupt.h"
#include "interrupt_table.h"
#include "v850_ins.h"
#include "device_io.h"

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

void interrupt_handler(unsigned int ecr)
{
	unsigned int intrno = ((ecr & 0x0000FFFF) - 0x80) >> 4;
	x_clear_int(intrno);
	enable_int_all();
	do_interrupt_handler(intrno);
	disable_int_all();
	return;
}
