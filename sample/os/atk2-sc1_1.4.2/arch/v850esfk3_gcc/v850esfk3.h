/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2014 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2014 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2014 by Witz Corporation, JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 *
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  $Id: v850.h 540 2015-12-29 01:00:01Z ertl-honda $
 */

/*
 *		V850のハードウェア資源の定義（開発環境共通）
 */

#ifndef TOPPERS_V850ESFK3_H
#define TOPPERS_V850ESFK3_H

/*
 * <TRACE> (8-4-1) TARGET_NAME
 *
 *  起動メッセージのターゲットシステム名
 */
#define TARGET_NAME	"V850ESFK3"

/*
 * 特定レジスタ
 */
#define CPU_SPC_PRCMD		0xFFFFF1FC

/*
 * プログラマブル周辺I/Oレジスタ
 */
#define CPU_IO_BPS			0xFFFFF064

/*
 * クロック
 */
#define CPU_CLOCK_CCLS		0xFFFFF82E
#define CPU_CLOCK_MCM		0xFFFFF860
#define CPU_CLOCK_OSTC		0xFFFFF6C2
#define CPU_CLOCK_OSTS		0xFFFFF6C0
#define CPU_CLOCK_PCC		0xFFFFF828
#define CPU_CLOCK_PCLM		0xFFFFF82F
#define CPU_CLOCK_RCM		0xFFFFF80C
#define CPU_CLOCK_LOCKR		0xFFFFF824
#define CPU_CLOCK_PLLCTL	0xFFFFF82C
#define CPU_CLOCK_PLLS		0xFFFFF6C1
#define CPU_CLOCK_PSC		0xFFFFF1FE
#define CPU_CLOCK_PSMR		0xFFFFF820
#define CPU_CLOCK_PRSM0		0xFFFFF8B0
#define CPU_CLOCK_PRSCM0	0xFFFFF8B1
#define CPU_CLOCK_CLM		0xFFFFF870
#define CPU_CLOCK_SELCNT0	0xFFFFF308
#define CPU_CLOCK_SELCNT1	0xFFFFF30A
#define CPU_CLOCK_SELCNT2	0xFFFFF30C
#define CPU_CLOCK_SELCNT3	0xFFFFF30E
#define CPU_CLOCK_SELCNT4	0xFFFFF3F8
#define CPU_CLOCK_SELCNT5	0xFFFFF3FA


#define IMR_SIZE			8U

#define TMIN_INTNO			0U
#define TMAX_INTNO			116U
/*
 * <TRACE> 7.1 設定ファイルとターゲット依存部の位置付け (b) 値取得シンボルテーブル
 *
 * 割込み番号54は欠番であるため，
 * 使用可能な総数は116個．
 */
#define TNUM_INT			117U

/*
 * <TRACE> 7.1 設定ファイルとターゲット依存部の位置付け (b) 値取得シンボルテーブル
 * <TRACE> (7-2-1-3) TNUM_INTPRI
 *
 * 割込み優先度の数
 */
#define TNUM_INTPRI			8

/*
 * 割込みコントローラ
 */
/*
 * IMR0-IMR7
 */
#define INTC_BASE	UINT_C(0xFFFFF110)
#define INTC_IMR0	UINT_C(0xFFFFF100)
#define INTC_IMR1	UINT_C(0xFFFFF102)
#define INTC_IMR2	UINT_C(0xFFFFF104)
#define INTC_IMR3	UINT_C(0xFFFFF106)
#define INTC_IMR4	UINT_C(0xFFFFF108)
#define INTC_IMR5	UINT_C(0xFFFFF10A)
#define INTC_IMR6	UINT_C(0xFFFFF10C)
#define INTC_IMR7	UINT_C(0xFFFFF10E)

/*
 * ISPR
 */
#define INTC_ISPR	UINT_C(0xFFFFF1FA)

/*
 * ウォッチドッグ・タイマ・モード・レジスタ2
 */
#define WDTM2	UINT_C(0xFFFFF6D0)

/*************************************************
 * 端子機能設定
 *************************************************/

/* common */
#define OCDM	UINT_C(0xFFFFF9FC)

#define PORT_GROUP_0	UINT_C(0)
#define PORT_GROUP_1	UINT_C(1)
#define PORT_GROUP_2	UINT_C(2)
#define PORT_GROUP_3	UINT_C(3)
#define PORT_GROUP_4	UINT_C(4)
#define PORT_GROUP_5	UINT_C(5)
#define PORT_GROUP_6	UINT_C(6)
#define PORT_GROUP_7	UINT_C(7)
#define PORT_GROUP_8	UINT_C(8)
#define PORT_GROUP_9	UINT_C(9)
#define PORT_GROUP_12	UINT_C(12)
#define PORT_GROUP_15	UINT_C(15)

#define PORT_BIT_0	UINT_C(0x01)
#define PORT_BIT_1	UINT_C(0x02)
#define PORT_BIT_2	UINT_C(0x04)
#define PORT_BIT_3	UINT_C(0x08)
#define PORT_BIT_4	UINT_C(0x10)
#define PORT_BIT_5	UINT_C(0x20)
#define PORT_BIT_6	UINT_C(0x40)
#define PORT_BIT_7	UINT_C(0x80)
#define PORT_BIT_8	UINT_C(0x0100)
#define PORT_BIT_9	UINT_C(0x0200)
#define PORT_BIT_10	UINT_C(0x0400)
#define PORT_BIT_11	UINT_C(0x0800)
#define PORT_BIT_12	UINT_C(0x1000)
#define PORT_BIT_13	UINT_C(0x2000)
#define PORT_BIT_14	UINT_C(0x4000)
#define PORT_BIT_15	UINT_C(0x8000)

#define IS_PORT_BIT_HIGH(reg, bit)	(((reg) & (bit)) && (bit))


/*
 * ポート・モード・コントロール・レジスタ
 */
#define PMCn_BASE		UINT_C(0xFFFFF440)
#define PMCn(group)		(PMCn_BASE + ((group) * 2U))
#define PMCnL(group)	(PMCn_BASE + ((group) * 2U))
#define PMCnH(group)	(PMCn_BASE + ((group) * 2U) + 1U)

#define PMCCM			UINT_C(0xFFFFF04C)
#define PMCCS			UINT_C(0xFFFFF048)
#define PMCCT			UINT_C(0xFFFFF04A)

#define PMCDLL			UINT_C(0xFFFFF044)
#define PMCDLH			UINT_C(0xFFFFF045)
#define PMCDL			UINT_C(0xFFFFF044)

/*
 * ポート・モード・レジスタ
 */
#define PMn_BASE		UINT_C(0xFFFFF420)
#define PMn(group) 		(PMn_BASE + ((group) * 2U))
#define PMnL(group)		(PMn_BASE + ((group) * 2U))
#define PMnH(group)		(PMn_BASE + ((group) * 2U) + 1U)

#define PMCD			UINT_C(0xFFFFF02E)
#define PMCM			UINT_C(0xFFFFF02C)
#define PMCS			UINT_C(0xFFFFF028)
#define PMCT			UINT_C(0xFFFFF02A)

#define PMDLL			UINT_C(0xFFFFF024)
#define PMDLH			UINT_C(0xFFFFF025)
#define PMDL			UINT_C(0xFFFFF024)

/*
 * ポート・ファンクション・コントロール・レジスタ
 */
#define PFCn_BASE		UINT_C(0xFFFFF460)
#define PFCn(group)		(PFCn_BASE + ((group) * 2U))
#define PFCnL(group)	(PFCn_BASE + ((group) * 2U))
#define PFCnH(group)	(PFCn_BASE + ((group) * 2U) + 1U)

/*
 * ポート・ファンクション・コントロール拡張レジスタ
 */
#define PFCEn_BASE		UINT_C(0xFFFFF700)
#define PFCEn(group)	(PFCEn_BASE + ((group) * 2U))
#define PFCEnL(group)	(PFCEn_BASE + ((group) * 2U))
#define PFCEnH(group)	(PFCEn_BASE + ((group) * 2U) + 1U)

/*
 * ポート・レジスタ
 */
#define Pn_BASE			UINT_C(0xFFFFF400)
#define Pn(group)		(Pn_BASE + ((group) * 2U))
#define PnL(group)		(Pn_BASE + ((group) * 2U))
#define PnH(group)		(Pn_BASE + ((group) * 2U) + 1U)

#define PCD				UINT_C(0xFFFFF00E)
#define PCM				UINT_C(0xFFFFF00C)
#define PCS				UINT_C(0xFFFFF008)
#define PCT				UINT_C(0xFFFFF00A)

#define PDLL			UINT_C(0xFFFFF004)
#define PDLH			UINT_C(0xFFFFF005)
#define PDL				UINT_C(0xFFFFF004)

/*
 * プルアップ抵抗オプション・レジスタ
 */
#define PUn_BASE		UINT_C(0xFFFFFC40)
#define PUn(group)		(PUn_BASE + ((group) * 2U))
#define PUnL(group)		(PUn_BASE + ((group) * 2U))
#define PUnH(group)		(PUn_BASE + ((group) * 2U) + 1U)

/*
 * オープン・ドレーン設定
 */
#define PF9H			UINT_C(0xFFFFFC73)


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


/*
 * アシンクロナス・シリアル・インタフェース（ UARTD）
 */
#define UDnChannelNum			UINT_C(8)
#define UDnCH0					UINT_C(0)
#define UDnCH1					UINT_C(1)
#define UDnCH2					UINT_C(2)
#define UDnCH3					UINT_C(3)
#define UDnCH4					UINT_C(4)
#define UDnCH5					UINT_C(5)
#define UDnCH6					UINT_C(6)
#define UDnCH7					UINT_C(7)

/*
 * UARTDn制御レジスタ 0（ UDnCTL0）
 */
#define UDnCTL0_BASE			UINT_C(0xFFFFFA00)
#define UDnCTL0(CH)				(UDnCTL0_BASE + ((CH) * 16U))

/*
 * UARTDn制御レジスタ 1（ UDnCTL1）
 */
#define UDnCTL1_BASE			UINT_C(0xFFFFFA01)
#define UDnCTL1(CH)				(UDnCTL1_BASE + ((CH) * 16U))
/*
 * UARTDn制御レジスタ 2（ UDnCTL2）
 */
#define UDnCTL2_BASE			UINT_C(0xFFFFFA02)
#define UDnCTL2(CH)				(UDnCTL2_BASE + ((CH) * 16U))

/*
 * UARTDn オプション制御レジスタ 0（ UDnOPT0）
 */
#define UDnOPT0_BASE			UINT_C(0xFFFFFA03)
#define UDnOPT0(CH)				(UDnOPT0_BASE + ((CH) * 16U))

/*
 * UARTDn オプション制御レジスタ 1（ UDnOPT1）
 */
#define UDnOPT1_BASE			UINT_C(0xFFFFFA05)
#define UDnOPT1(CH)				(UDnOPT1_BASE + ((CH) * 16U))

/*
 * UARTDn状態レジスタ（ UDnSTR）
 */
#define UDnSTR_BASE				UINT_C(0xFFFFFA04)
#define UDnSTR(CH)				(UDnSTR_BASE + ((CH) * 16U))

/*
 * UARTDn送信データ・レジスタ（ UDnTX）
 */
#define UDnTX_BASE				UINT_C(0xFFFFFA07)
#define UDnTX(CH)				(UDnTX_BASE + ((CH) * 16U))

/*
 * UARTDn受信データ・レジスタ（ UDnRX）
 */
#define UDnRX_BASE				UINT_C(0xFFFFFA06)
#define UDnRX(CH)				(UDnRX_BASE + ((CH) * 16U))

/*
 *  V850ESFK3用の割込みコントローラ操作ルーチン
 */
 /* intno は unsigned を想定 */
#define EIC_ADDRESS(intno)	(INTC_BASE + (intno * 2U)) 

#ifndef TOPPERS_MACRO_ONLY

#include "prc_sil.h"


#endif /* TOPPERS_MACRO_ONLY */


#endif /* TOPPERS_V850_H */
