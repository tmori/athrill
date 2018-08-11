

#include "kernel_impl.h"
#include <sil.h>
#include "v850es_fk3_emu_env.h"


static void target_fput_initialize(void);


void
target_initialize(void)
{
	uint16_t wr_mem_h;
	uint32_t wr_mem_w;


	prc_initialize();


	target_fput_initialize();


	clr_bit(LED1_BITPOS , LED1_ADDRESS);
	clr_bit(LED1_BITPOS , PCMT);


	wr_mem_h = (sil_reh_mem((void *)PMC3L) & 0xfffc) | 0x003;
	sil_wrh_mem((void *)PMC3L, wr_mem_h);
	wr_mem_w = (sil_reh_mem((void *)PFC3) & 0xfffc);
	sil_wrh_mem((void *)PFC3, wr_mem_w);
}


void
target_exit(void)
{
	volatile uint_t i = 1;


	prc_terminate();


	while(i)
		;
}

static void target_fput_initialize(void)
{

	clr_bit(7 , UA0CTL0);


	sil_wrb_mem((void *)UA0CTL1 , TARGET_FPUTC_UAnCTL1_SETTING);
	sil_wrb_mem((void *)UA0CTL2 , TARGET_FPUTC_UAnCTL2_SETTING);


	sil_wrb_mem((void *)UA0CTL0 , 0xD2);
}


void
target_fput_log(char c)
{
	if (c == '\n') {
		while((sil_reb_mem((void *)TARGET_FPUTC_UAnSTR) & 0x80) != 0x00)
			;
		sil_wrb_mem((void *)TARGET_FPUTC_UAnTX , '\r');
	}
	while((sil_reb_mem((void *)TARGET_FPUTC_UAnSTR) & 0x80) != 0x00)
		;
	sil_wrb_mem((void *)TARGET_FPUTC_UAnTX , c);
}
