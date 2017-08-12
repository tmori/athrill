#ifndef _TIMER_H_
#define _TIMER_H_

#include "device.h"

/*************************************************
 * 16ビットタイマ／イベントカウンタAA(TAA)
 *************************************************/

#define TAAnChannelNum			UINT_C(8)
#define TAAnCH0					UINT_C(0)
#define TAAnCH1					UINT_C(1)
#define TAAnCH2					UINT_C(2)
#define TAAnCH3					UINT_C(3)
#define TAAnCH4					UINT_C(4)
#define TAAnCH5					UINT_C(5)
#define TAAnCH6					UINT_C(6)
#define TAAnCH7					UINT_C(7)

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



#endif /* _TIMER_H_ */
