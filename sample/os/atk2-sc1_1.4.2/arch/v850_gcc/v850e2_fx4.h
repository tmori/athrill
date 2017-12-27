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
 *  $Id: v850e2_fx4.h 540 2015-12-29 01:00:01Z ertl-honda $
 */

/*
 *		V850E2/Fx4のハードウェア資源の定義
 */

#ifndef TOPPERS_V850E2_FX4_H
#define TOPPERS_V850E2_FX4_H

#if defined(V850FG4) || defined(V850FL4)
#define _V850E2M_
#elif defined(V850FG4_L)
#define _V850E2S_
#endif /* V850FG4 */

/*
 *  PORTレジスタ
 */
#define PORT_BASE	UINT_C(0xff400000)

/* 端子機能設定 */
#define PMC(n)		((PORT_BASE) +0x0400 + (n * 0x04U))     /* ポート・モード・コントロール・レジスタ */
#define PMCSR(n)	((PORT_BASE) +0x0900 + (n * 0x04U))     /* ポート・モード・コントロール・セット／リセット・レジスタ */
#define PIPC(n)		((PORT_BASE) +0x4200 + (n * 0x04U))     /* ポートIP コントロール・レジスタ */
#define PM(n)		((PORT_BASE) +0x0300 + (n * 0x04U))     /* ポート・モード・レジスタ */
#define PMSR(n)		((PORT_BASE) +0x0800 + (n * 0x04U))     /* ポート・モード・セット／リセット・レジスタ */
#define PIBC(n)		((PORT_BASE) +0x4000 + (n * 0x04U))     /* ポート入力バッファ・コントロール・レジスタ */
#define PFC(n)		((PORT_BASE) +0x0500 + (n * 0x04U))     /* ポート機能コントロール・レジスタ */
#define PFCE(n)		((PORT_BASE) +0x0600 + (n * 0x04U))     /* ポート機能コントロール・レジスタ */

/* 端子データ入力／出力 */
#define PBDC(n)		((PORT_BASE) +0x4100 + (n * 0x04U))     /* ポート双方向コントロール・レジスタ */
#define PPR(n)		((PORT_BASE) +0x0200 + (n * 0x04U))     /* ポート端子リード・レジスタ */
#define P(n)		((PORT_BASE) +0x0000 + (n * 0x04U))     /* ポート・レジスタ */
#define PNOT(n)		((PORT_BASE) +0x0700 + (n * 0x04U))     /* ポート・ノット・レジスタ */
#define PSR(n)		((PORT_BASE) +0x0100 + (n * 0x04U))     /* ポート・セット／リセット・レジスタ */

#define FCLA27CTL1	0xff416244      /* UARTE3フィルタレジスタ */
#define FCLA27CTL3	0xff41624c
#define FCLA7CTL0	0xff415040
#define FCLA0CTL2	0xff414008
#define FCLA0CTL4	0xff414010

/*
 *  PLL関連のレジスタと定義
 */
#define MOSCC		0xff421018
#define MOSCE		0xff421010
#define MOSCS		0xff421014
#define MOSCST		0xff42101c
#define OCDIDH		0xff470008
#define OCDIDL		0xff470000
#define OCDIDM		0xff470004
#define OPBT0		0xff47000c
#define OSCWUFMSK	0xff4201a4

#define SOSCE		0xff421020
#define SOSCS		0xff421024
#define SOSCST		0xff42102c

#define CKSC_0_BASE		0xff426000
#define CKSC_0(n)		(CKSC_0_BASE + (0x10 * n))

#define CKSC_1_BASE		0xff42a000
#define CKSC_1(n)		(CKSC_1_BASE + (0x10 * n))

#define CKSC_A_BASE		0xff422000
#define CKSC_A(n)		(CKSC_A_BASE + (0x10 * n))

#define CSCSTAT_0_BASE	0xff426000
#define CSCSTAT_0(n)	(CSCSTAT_0_BASE + (0x10 * n)) + 4

#define CSCSTAT_1_BASE	0xff42a000
#define CSCSTAT_1(n)	(CSCSTAT_1_BASE + (0x10 * n)) + 4

#define CSCSTAT_A_BASE	0xff422000
#define CSCSTAT_A(n)	(CSCSTAT_A_BASE + (0x10 * n)) + 4

#define PLLE_BASE	0xff425000
#define PLLE(n)		(PLLE_BASE + (0x10 * n))

#define PLLS_BASE	0xff425004
#define PLLS(n)		(PLLS_BASE + (0x10 * n))

#define PLLC_BASE	0xff425008
#define PLLC(n)		(PLLC_BASE + (0x10 * n))

#define PLLST_BASE	0xff42500c
#define PLLST(n)	(PLLST_BASE + (0x10 * n))

#define MHz(n)		((n) * 1000 * 1000)
#define CLK_MHz(num)	(num * 1000 * 1000)

/* CKSC_000 CPU, CPU SubSystem */
#define HIGH_SPEED_INTOSC_DIV2	0x08
#define HIGH_SPEED_INTOSC_DIV4	0x09
#define HIGH_SPEED_INTOSC_DIV8	0x0A
#define HIGH_SPEED_INTOSC_DIV32	0x0B
#define MAINOSC_DIV1			0x0C
#define PLL0_DIV1				0x14
#define PLL0_DIV2				0x15
#define PLL0_DIV3				0x16
#define PLL0_DIV4				0x17
#define PLL0_DIV5				0x18
#define PLL0_DIV6				0x19
#define PLL0_DIV8				0x1A
#define PLL0_DIV10				0x1B
#define INTOSC_AUTOSELECT		0x3A

/* CKSC_A03 TAUJ0:PCLK */
#define LOW_SPEED_INTOSC_DIV1	0x01
#define HIGH_SPEED_INTOSC_DIV1	0x07
#define SUBOSC					0x12
#define NO_CLOCKSELECT			0x00

#define PROT_PLLE	2 /*  Fx4 Setting */
#define PROT_CKSC0	0
#define PROT_CKSC1	1
#define PROT_MOSCE	2
#define PROT_SOSCE	2
#define PROT_ROSCE	2
#define PROT_CKSCA	2

/* xxxS Register */
#define CLK_S_STPACK	0x08
#define CLK_S_CLKEN		0x04
#define CLK_S_CLKACT	0x02
#define CLK_S_CLKSTAB	0x01

/* PLL P-Value */
#define PDIV0R5_200TO400	0x0     /* Div 0.5, 200-400MHz Output */
#define PDIV1R0_100TO200	0x1     /* Div 1.0, 100-200MHz Output */
#define PDIV2R0_050TO100	0x2     /* Div 2.0,  50-100MHz Output */
#define PDIV4R0_025TO050	0x3     /* Div 4.0,  25- 50MHz Output */

#define UC_SUCCESS			0
#define UC_ERROR			1
#define UC_INVALIDPARAM		2
#define UC_PROTREGERROR		3
#define UC_CLKSTATUSERR		4
#define UC_CLKNOTENABLE		5
#define UC_CLKNOTACTIVE		6
#define UC_CLKNOTSTAB		7

#ifndef TOPPERS_MACRO_ONLY

extern uint32 EnableSubOSC(void);

extern uint32 EnableMainOSC(uint32 clk_in);

extern uint32 SetPLL(uint32 pllno, uint32 mhz, uint32 *outclk);

extern uint32 set_clock_selection(uint32 control, uint32 status, uint8 regno, uint16 sel);

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  Interval Timer(TAUA0)
 */
#define TAUA0_BASE			UINT_C(0xFF808000)  /* TAUA0 */

#define TAUA0_IRQ			UINT_C(20)          /* TAUA0 */
#define TAUA1_IRQ			UINT_C(21)          /* TAUA1 */
#define TAUA2_IRQ			UINT_C(22)          /* TAUA2 */
#define TAUA3_IRQ			UINT_C(23)          /* TAUA3 */
#define TAUA4_IRQ			UINT_C(24)          /* TAUA4 */
#define TAUA5_IRQ			UINT_C(25)          /* TAUA5 */
#define TAUA6_IRQ			UINT_C(26)          /* TAUA6 */
#define TAUA7_IRQ			UINT_C(27)          /* TAUA7 */
#define TAUA8_IRQ			UINT_C(28)          /* TAUA8 */
#define TAUA9_IRQ			UINT_C(29)          /* TAUA9 */
#define TAUA10_IRQ			UINT_C(30)          /* TAUA10 */
#define TAUA11_IRQ			UINT_C(31)          /* TAUA11 */
#define TAUA12_IRQ			UINT_C(32)          /* TAUA12 */
#define TAUA13_IRQ			UINT_C(33)          /* TAUA13 */
#define TAUA14_IRQ			UINT_C(34)          /* TAUA14 */
#define TAUA15_IRQ			UINT_C(35)          /* TAUA15 */

#define TAUA_CH0			0
#define TAUA_CH1			1
#define TAUA_CH2			2
#define TAUA_CH3			3
#define TAUA_CH4			4
#define TAUA_CH5			5
#define TAUA_CH6			6
#define TAUA_CH7			7
#define TAUA_CH8			8
#define TAUA_CH9			9
#define TAUA_CH10			10
#define TAUA_CH11			11
#define TAUA_CH12			12
#define TAUA_CH13			13
#define TAUA_CH14			14
#define TAUA_CH15			15

/*
 *  TAUA0 Timer ハードウェア定義
 */

/*
 *  レジスタ
 */
/* TAUA0 プリスケーラ・レジスタ */
#define TAUA0TPS		(TAUA0_BASE + 0x240U)               /* プリスケーラ・クロック選択レジス */
#define TAUA0BRS		(TAUA0_BASE + 0x244U)               /* プリスケーラ・ボー・レート設定レジスタ */

/* TAUA0 制御レジスタ */
#define TAUA0CDR(CH)	(TAUA0_BASE + (CH * 4U))            /* データ・レジスタ */
#define TAUA0CNT(CH)	(TAUA0_BASE + (0x80U + (CH * 4U)))  /* カウンタ・レジスタ */
#define TAUA0CMOR(CH)	(TAUA0_BASE + (0x200U + (CH * 4U))) /* モードOS レジスタ */
#define TAUA0CMUR(CH)	(TAUA0_BASE + (0xC0 + (CH * 4U)))   /* モード・ユーザ・レジスタ */
#define TAUA0CSR(CH)	(TAUA0_BASE + (0x140U + (CH * 4U))) /* ステータス・レジスタ */
#define TATA0CSC(CH)	(TAUA0_BASE + (0x180U + (CH * 4U))) /* ステータス・クリア・トリガ・レジスタ */
#define TAUA0TS		(TAUA0_BASE + 0x1C4U)                   /* スタート・トリガ・レジスタ */
#define TAUA0TE		(TAUA0_BASE + 0x1C0U)                   /* 許可ステータス・レジスタ */
#define TAUA0TT		(TAUA0_BASE + 0x1C8U)                   /* ストップ・トリガ・レジスタ */

/* TAUA0 出力レジスタ */
#define TAUA0TOE	(TAUA0_BASE + 0x5CU)                    /* 出力許可レジスタ */
#define TAUA0TO		(TAUA0_BASE + 0x58U)                    /* 出力レジスタ */
#define TAUA0TOM	(TAUA0_BASE + 0x248U)                   /* 出力モード・レジスタ */
#define TAUA0TOC	(TAUA0_BASE + 0x24CU)                   /* 出力コンフィギュレーション・レジスタ */

#define TAUA0TOL	(TAUA0_BASE + 0x40U)                    /* 出力アクティブ・レベル・レジスタ */
#define TAUA0TDE	(TAUA0_BASE + 0x250U)                   /* デッド・タイム出力許可レジスタ */
#define TAUA0TDM	(TAUA0_BASE + 0x254U)                   /* デッド・タイム出力モード・レジスタ */
#define TAUA0TDL	(TAUA0_BASE + 0x54U)                    /* デッド・タイム出力レベル・レジスタ */

#define TAUA0TRO	(TAUA0_BASE + 0x4CU)                    /* リアルタイム出力レジスタ */
#define TAUA0TRE	(TAUA0_BASE + 0x258U)                   /* リアルタイム出力許可レジスタ */
#define TAUA0TRC	(TAUA0_BASE + 0x25CU)                   /* リアルタイム出力制御レジスタ */
#define TAUA0TME	(TAUA0_BASE + 0x50U)                    /* 変調出力許可レジスタ */
/* TAUA0 リロード・データ・レジスタ */
#define TAUA0RDE	(TAUA0_BASE + 0x260U)                   /* リロード・データ許可レジスタ */
#define TAUA0RDM	(TAUA0_BASE + 0x264U)                   /* リロード・データ・モード・レジスタ */
#define TAUA0RDS	(TAUA0_BASE + 0x268U)                   /* リロード・データ制御CH 選択・リロード・データ制御CH 選択 */
#define TAUA0RDC	(TAUA0_BASE + 0x26CU)                   /* リロード・データ制御レジスタ */
#define TAUA0RDT	(TAUA0_BASE + 0x44U)                    /* リロード・データ・トリガ・レジスタ */
#define TAUA0RSF	(TAUA0_BASE + 0x48U)                    /* リロード・ステータス・レジスタ */

#define MCU_TAUA0_MASK_CK0				((uint16) 0x000f)
#define MCU_TAUA0_CK0					((uint16) 0x0000)
#define MCU_TAUA00_CMOR					((uint16) 0x0001)
#define MCU_TAUA00_CMUR					((uint8) 0x01)
#define MCU_TAUA00_DI					((uint16) 0x0080)
#define MCU_TAUA00_EI					((uint16) 0x0000)
#define MCU_TAUA00_MASK_ENB				((uint16) 0x0001)
#define MCU_TIMER_STOP					((uint8) 0x0)
#define MCU_TIMER_START					((uint8) 0x1)

#define ICTAUA0_BASE					0xffff6028                  /* チャンネル０割り込み */
#define ICTAUA0I(CH)					(ICTAUA0_BASE + (CH * 0x02))


/*
 *  TAUA0 マスク定義
 */
#define TAUA0_MASK_BIT	0x0xfffe                                    /* bit0 = TAUA0 */

/*
 * OSTM
 */
#define OSTM_IRQ			UINT_C(147)

#define OSTM0_BASE	0xFF800000

#define OSTM_CMP_W	(0xFF800000 + 0x00)
#define OSTM_CNT_W	(0xFF800000 + 0x04)
#define OSTM_TE		(0xFF800000 + 0x10)
#define OSTM_TS_B	(0xFF800000 + 0x14)
#define OSTM_TT_B	(0xFF800000 + 0x18)
#define OSTM_CTL_B	(0xFF800000 + 0x20)

/*
 *  UARTE
 */
#define URTE3_BASE	UINT_C(0xff5f0000)
#define URTE5_BASE	UINT_C(0xff610000)
#define URTE10_BASE	UINT_C(0xff660000)
#define URTEnCTL0	(UARTE_BASE + 0x00U)
#define URTEnCTL1	(UARTE_BASE + 0x20U)
#define URTEnCTL2	(UARTE_BASE + 0x24U)
#define URTEnTRG	(UARTE_BASE + 0x04U)
#define URTEnSTR0	(UARTE_BASE + 0x08U)
#define URTEnSTR1	(UARTE_BASE + 0x0cU)
#define URTEnSTC	(UARTE_BASE + 0x10U)
#define URTEnRX		(UARTE_BASE + 0x14U)
#define URTEnTX		(UARTE_BASE + 0x18U)
#define URTEnEMU	(UARTE_BASE + 0x34U)

#define INTLMA3IT	0xffff618C      /* 転送完了 */
#define INTLMA3IR	0xffff618A      /* 受信完了 */

#define INTLMA5IT	0xffff61C4      /* UART5 RX */
#define INTLMA5IR	0xffff61C6      /* UART5 TX */

#define INTLMA10IT	0xffff6204      /* UART10 TX */
#define INTLMA10IR	0xffff6202      /* UART10 RX */

#define URTE3_INTNO		UINT_C(197)
#define URTE5_INTNO		UINT_C(226)

#ifndef URTE10_INTNO
#define URTE10_INTNO	UINT_C(249)
#endif /* URTE10_INTNO */

/*
 *  TAUJ
 */
#ifdef V850FG4
#define TAUFJ0I0_INTNO	135        
#define TAUFJ0I1_INTNO	136
#define TAUFJ0I2_INTNO	137
#define TAUFJ0I3_INTNO	138
#define TAUFJ1I0_INTNO	139
#define TAUFJ1I1_INTNO	140
#define TAUFJ1I2_INTNO	141
#define TAUFJ1I3_INTNO	142
#elif defined(V850FG4_L)
#define TAUFJ0I0_INTNO	78
#define TAUFJ0I1_INTNO	79
#define TAUFJ0I2_INTNO	80
#define TAUFJ0I3_INTNO	81
#endif /* V850FG4 */


/*
 *  TAUJ関連レジスタ
 */
#define TAUJ_BASE(n)	((uint32) (0xff811000U + (n * 0x1000U)))
#define TAUJTPS(n)		(TAUJ_BASE(n) + 0x90U)
#define TAUJCDR(n, ch)	(TAUJ_BASE(n) + (ch * 0x04U))
#define TAUJCNT(n, ch)	(TAUJ_BASE(n) + 0x10U + (ch * 0x04U))
#define TAUJCMOR(n, ch)	(TAUJ_BASE(n) + 0x80U + (ch * 0x04U))
#define TAUJCMUR(n, ch)	(TAUJ_BASE(n) + 0x20U + (ch * 0x04U))
#define TAUJTS(n)		(TAUJ_BASE(n) + 0x54U)
#define TAUJTT(n)		(TAUJ_BASE(n) + 0x58U)

/*
 *  INT
 */
#define EIC_BASE			UINT_C(0xffff6000)
#define EIC_ADDRESS(intno)	(EIC_BASE + (intno * 2))
#define PMR					UINT_C(0xFFFF6448)
#define ISPR_H				UINT_C(0xFFFF6440)
#define ISPC_H				UINT_C(0xffff6450)

#define TMIN_INTNO	UINT_C(0)
#define TMAX_INTNO	UINT_C(255)
#define TNUM_INT	UINT_C(256)

#include "v850.h"

#endif /* TOPPERS_V850E2_FX4_H */
