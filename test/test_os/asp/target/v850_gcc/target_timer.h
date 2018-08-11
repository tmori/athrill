

#ifndef TARGET_TIMER_H
#define TARGET_TIMER_H

#include <sil.h>
#include "v850es_fk3_emu_env.h"


#define TIMER_CLOCK (1000)


#define INHNO_TIMER 	22
#define INTNO_TIMER 	22
#define INTPRI_TIMER	(-6)
#define INTATR_TIMER	TA_NULL


typedef uint32_t    CLOCK;


#define TO_CLOCK(nume, deno)    ((CLOCK)(TIMER_CLOCK * (nume) / (deno)))
#define TO_USEC(clock)          (((SYSUTM) clock) * 1000U / TIMER_CLOCK)


#define MAX_CLOCK    ((CLOCK) 0xffffU)

#ifndef TOPPERS_MACRO_ONLY

extern void target_timer_initialize(intptr_t exinf);


extern void target_timer_terminate(intptr_t exinf);


#define target_timer_get_current()	(0)


Inline bool_t target_timer_probe_int(void)
{
	return x_probe_int(INTNO_TIMER);
}


extern void target_timer_handler(void);

#endif	/* TOPPERS_MACRO_ONLY */

/*
 * TAAn制御レジスタ0
 */
#define TAAnCTL0_BASE			UINT_C(0xFFFFF590)
#define TAAnCTL0(CH)			(TAAnCTL0_BASE + ((CH) * 16U))
/*
 * TAAn制御レジスタ1
 */
#define TAAnCTL1_BASE			UINT_C(0xFFFFF591)
#define TAAnCTL1(CH)			(TAAnCTL1_BASE + ((CH) * 16U))

/*
 * TAAn キャプチャ／コンペア・レジスタ 0（ TAAnCCR0）
 */
#define TAAnCCR0_BASE			UINT_C(0xFFFFF596)
#define TAAnCCR0(CH)			(TAAnCCR0_BASE + ((CH) * 16U))

/*
 * TAAn キャプチャ／コンペア・レジスタ 1（ TAAnCCR1）
 */
#define TAAnCCR1_BASE			UINT_C(0xFFFFF598)
#define TAAnCCR1(CH)			(TAAnCCR1_BASE + ((CH) * 16U))

/*
 * TAAnカウンタ・リード・バッファ・レジスタ
 */
#define TAAnCNT_BASE			UINT_C(0xFFFFF59A)
#define TAAnCNT(CH)				(TAAnCNT_BASE + ((CH) * 16U))

/*
 * TAAn オプション・レジスタ 0（ TAAnOPT0）
 */
#define TAAnOPT0_BASE			UINT_C(0xFFFFF595)
#define TAAnOPT0(CH)			(TAAnOPT0_BASE + ((CH) * 16U))

/*
 * TAAn オプション・レジスタ 1（ TAAnOPT1）
 */
#define TAA1OPT1				UINT_C(0xFFFFF5AD)
#define TAA3OPT1				UINT_C(0xFFFFF5CD)
#define TAA6OPT1				UINT_C(0xFFFFF5FD)
/*
 *  使用する差分タイマと現在値タイマのチャネル
 */
#define TIMER_DTIM_ID    TAAnCH2	/* 差分タイマ */
#define TIMER_CTIM_ID    TAAnCH3	/* 現在値タイマ */



/*
 *  割込み要求のクリア
 */
Inline void
HwcounterClearInterrupt(uint32_t intno)
{
	(void)x_clear_int(intno);
}
/*
 *  割込み禁止／許可設定
 */
Inline void
HwcounterDisableInterrupt(uint32_t intno)
{
	(void)x_disable_int(intno);
}

Inline void
HwcounterEnableInterrupt(uint32_t intno)
{
	(void)x_enable_int(intno);
}
 /*
 *  TAA タイマの動作開始／停止処理
 */
Inline void
SetTimerStartTAA(uint8_t ch)
{
	/* タイマ開始処理 */
	sil_wrb_mem((void *) TAAnCTL0(ch),
		( sil_reb_mem((void *) TAAnCTL0(ch)) | (1U << 7U) )	//TAAnCEビットセット
	);
}

Inline void
SetTimerStopTAA(uint8_t ch)
{
	/* タイマ停止処理 */
	sil_wrb_mem((void *) TAAnCTL0(ch),
		( sil_reb_mem((void *) TAAnCTL0(ch)) & ~(1U << 7U) )	//TAAnCEビットクリア
	);
}

/*
 *  TAAハードウェアカウンタ現在ティック値取得
 */
Inline uint16_t
GetCurrentTimeTAA(uint8_t ch)
{
	uint16_t	count;

	count = sil_reh_mem((void *) (TAAnCNT(ch)));
	return(count);
}

#endif	/* TARGET_TIMER_H */
