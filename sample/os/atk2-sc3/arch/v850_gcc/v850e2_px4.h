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
 *  上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: v850e2_px4.h 117 2014-12-10 03:58:03Z t_ishikawa $
 */

/*
 *		V850E2/Px4のハードウェア資源の定義
 */

#ifndef TOPPERS_V850E2_PX4_H
#define TOPPERS_V850E2_PX4_H

#define _V850E2M_

#define VPNECR	0xffff5110
#define VPNADR	0xffff5114
#define VPNTID	0xffff5118
#define VPTTID	0xffff511A
#define VPTECR	0xffff5120
#define VPTADR	0xffff5124

/*
 *  ポートレジスタ
 */
#define PM0		0xffff8300
#define PMC0	0xffff8400
#define PFC0	0xffff8500
#define PFCE0	0xffff8600

#define P4		0xffff8010
#define PM4		0xffff8310
#define PMC4	0xffff8410
#define PFC4	0xffff8510
#define PFCE4	0xffff8610

#define FCLA0CTL0	0xFF414000
#define FCLA1CTL2	0xFF414028
#define FCLA1CTL3	0xFF41402c

#define FCLA27CTL3	0xff41624c
#define FCLA27CTL6	0xff416258

/*
 *  Interval Timer(TAUA0)
 */
#define TAUA0_BASE0			UINT_C(0xFF808000)  /* TAUA0 */
#define TAUA0_BASE1			UINT_C(0xFFFFC400)  /* TAUA0 */

#define TAUA0_IRQ			UINT_C(54)          /* TAUA0 */
#define TAUA1_IRQ			UINT_C(55)          /* TAUA1 */
#define TAUA2_IRQ			UINT_C(56)          /* TAUA2 */
#define TAUA3_IRQ			UINT_C(57)          /* TAUA3 */
#define TAUA4_IRQ			UINT_C(58)          /* TAUA4 */
#define TAUA5_IRQ			UINT_C(59)          /* TAUA5 */
#define TAUA6_IRQ			UINT_C(60)          /* TAUA6 */
#define TAUA7_IRQ			UINT_C(61)          /* TAUA7 */
#define TAUA8_IRQ			UINT_C(62)          /* TAUA8 */
#define TAUA9_IRQ			UINT_C(63)          /* TAUA9 */
#define TAUA10_IRQ			UINT_C(64)          /* TAUA10 */
#define TAUA11_IRQ			UINT_C(65)          /* TAUA11 */
#define TAUA12_IRQ			UINT_C(66)          /* TAUA12 */
#define TAUA13_IRQ			UINT_C(67)          /* TAUA13 */
#define TAUA14_IRQ			UINT_C(68)          /* TAUA14 */
#define TAUA15_IRQ			UINT_C(69)          /* TAUA15 */

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
#define TAUA0TPS		(TAUA0_BASE0 + 0x240U)              /* プリスケーラ・クロック選択レジス */
#define TAUA0BRS		(TAUA0_BASE0 + 0x244U)              /* プリスケーラ・ボー・レート設定レジスタ */

/* TAUA0 制御レジスタ */
#define TAUA0CDR(CH)	(TAUA0_BASE1 + (CH * 4U))               /* データ・レジスタ */
#define TAUA0CNT(CH)	(TAUA0_BASE1 + (0x80U + (CH * 4U)))     /* カウンタ・レジスタ */
#define TAUA0CMOR(CH)	(TAUA0_BASE0 + (0x200U + (CH * 4U)))    /* モードOS レジスタ */
#define TAUA0CMUR(CH)	(TAUA0_BASE1 + (0xC0 + (CH * 4U)))      /* モード・ユーザ・レジスタ */
#define TAUA0CSR(CH)	(TAUA0_BASE1 + (0x140U + (CH * 4U)))    /* ステータス・レジスタ */
#define TATA0CSC(CH)	(TAUA0_BASE1 + (0x180U + (CH * 4U)))    /* ステータス・クリア・トリガ・レジスタ */
#define TAUA0TS		(TAUA0_BASE1 + 0x1C4U)                      /* スタート・トリガ・レジスタ */
#define TAUA0TE		(TAUA0_BASE1 + 0x1C0U)                      /* 許可ステータス・レジスタ */
#define TAUA0TT		(TAUA0_BASE1 + 0x1C8U)                      /* ストップ・トリガ・レジスタ */

/* TAUA0 出力レジスタ */
#define TAUA0TOE	(TAUA0_BASE1 + 0x5CU)                   /* 出力許可レジスタ */
#define TAUA0TO		(TAUA0_BASE1 + 0x58U)                   /* 出力レジスタ */
#define TAUA0TOM	(TAUA0_BASE0 + 0x248U)                  /* 出力モード・レジスタ */
#define TAUA0TOC	(TAUA0_BASE0 + 0x24CU)                  /* 出力コンフィギュレーション・レジスタ */

#define TAUA0TOL	(TAUA0_BASE1 + 0x40U)                   /* 出力アクティブ・レベル・レジスタ */
#define TAUA0TDE	(TAUA0_BASE0 + 0x250U)                  /* デッド・タイム出力許可レジスタ */
#define TAUA0TDM	(TAUA0_BASE0 + 0x254U)                  /* デッド・タイム出力モード・レジスタ */
#define TAUA0TDL	(TAUA0_BASE1 + 0x54U)                   /* デッド・タイム出力レベル・レジスタ */

#define TAUA0TRO	(TAUA0_BASE1 + 0x4CU)                   /* リアルタイム出力レジスタ */
#define TAUA0TRE	(TAUA0_BASE0 + 0x258U)                  /* リアルタイム出力許可レジスタ */
#define TAUA0TRC	(TAUA0_BASE0 + 0x25CU)                  /* リアルタイム出力制御レジスタ */
#define TAUA0TME	(TAUA0_BASE1 + 0x50U)                   /* 変調出力許可レジスタ */
/* TAUA0 リロード・データ・レジスタ */
#define TAUA0RDE	(TAUA0_BASE0 + 0x260U)                  /* リロード・データ許可レジスタ */
#define TAUA0RDM	(TAUA0_BASE0 + 0x264U)                  /* リロード・データ・モード・レジスタ */
#define TAUA0RDS	(TAUA0_BASE0 + 0x268U)                  /* リロード・データ制御CH 選択・リロード・データ制御CH 選択 */
#define TAUA0RDC	(TAUA0_BASE0 + 0x26CU)                  /* リロード・データ制御レジスタ */
#define TAUA0RDT	(TAUA0_BASE1 + 0x44U)                   /* リロード・データ・トリガ・レジスタ */
#define TAUA0RSF	(TAUA0_BASE1 + 0x48U)                   /* リロード・ステータス・レジスタ */

#define MCU_TAUA0_MASK_CK0				((uint16) 0x000f)
#define MCU_TAUA0_CK0					((uint16) 0x0000) /* 2^0 */
#define MCU_TAUA00_CMOR					((uint16) 0x0001)
#define MCU_TAUA00_CMUR					((uint8) 0x01)
#define MCU_TAUA00_DI					((uint16) 0x0080)
#define MCU_TAUA00_EI					((uint16) 0x0000)
#define MCU_TAUA00_MASK_ENB				((uint16) 0x0001)
#define MCU_TIMER_STOP					((uint8) 0x0)
#define MCU_TIMER_START					((uint8) 0x1)

#define ICTAUA0_BASE					0xffff606c                  /* チャンネル０割り込み */
#define ICTAUA0I(CH)					(ICTAUA0_BASE + (CH * 0x02))

/*
 *  TAUA0 マスク定義
 */
#define TAUA0_MASK_BIT	0x0xfffe                                    /* bit0 = TAUA0 */

/*
 *  UARTE
 */
#define URTE0_BASE	0xFF5C0000
#define URTE1_BASE	0xFF5D0000
#define URTE2_BASE0	0xFF5E0000
#define URTE2_BASE1	0xFFFFEC00

#define URTEnCTL0	(URTE2_BASE1 + 0x00U)
#define URTEnCTL1	(URTE2_BASE0 + 0x40U)
#define URTEnCTL2	(URTE2_BASE0 + 0x44U)
#define URTEnTRG	(URTE2_BASE1 + 0x0cU)
#define URTEnSTR0	(URTE2_BASE1 + 0x10U)
#define URTEnSTR1	(URTE2_BASE1 + 0x14U)
#define URTEnSTC	(URTE2_BASE1 + 0x18U)
#define URTEnRX		(URTE2_BASE1 + 0x1cU)
#define URTEnTX		(URTE2_BASE1 + 0x2cU)

#define URTE2_INTNO	UINT_C(197)

#define INTNO_URTE2_IS	114
#define INTNO_URTE2_IR	115
#define INTNO_URTE2_IT	116

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

#endif /* TOPPERS_V850E2_PX4_H */
