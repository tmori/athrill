

#include "kernel_impl.h"
#include <sil.h>
#include "v850es_fk3_emu_env.h"
#include "target_config.h"

/*
 *  システムログの低レベル出力のための初期化
 *
 *  セルタイプtPutLogV850ESFK3内に実装されている関数を直接呼び出す．
 */
extern void	tPutLogV850ESFK3_initialize(void);

static void target_fput_initialize(void);

void
target_initialize(void)
{
	uint16_t wr_mem_h;
	uint32_t wr_mem_w;


	prc_initialize();

	clr_bit(LED1_BITPOS , LED1_ADDRESS);
	clr_bit(LED1_BITPOS , PCMT);


	wr_mem_h = (sil_reh_mem((void *)PMC3L) & 0xfffc) | 0x003;
	sil_wrh_mem((void *)PMC3L, wr_mem_h);
	wr_mem_w = (sil_reh_mem((void *)PFC3) & 0xfffc);
	sil_wrh_mem((void *)PFC3, wr_mem_w);

	/*
	 *  低レベル出力用にSIOを初期化
	 */
#ifndef TOPPERS_OMIT_TECS
	tPutLogV850ESFK3_initialize();
#endif /* TOPPERS_OMIT_TECS */
}


void
target_exit(void)
{
	volatile uint_t i = 1;


	prc_terminate();


	while(i)
		;

}
