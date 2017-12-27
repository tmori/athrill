/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: fl850f1l.h 35 2014-07-17 14:00:37Z ertl-honda $
 */
/*
 *		RH850F1H_PBボードの定義
 */

#ifndef TOPPERS_RH850F1H_PB_H
#define TOPPERS_RH850F1H_PB_H

#include "v850_gcc/rh850_f1h.h"

/*
 *  PLL関連の定義
 */
#define MAINOSC_CLOCK_MHZ	16    /* Main OSC is 16MHz */

#define PLL0_CLK_MHZ	120 /* PLL is 120MHz */

/* PPLLCLK(120MHz)=16MHz x (Nr(60)/Mr(2)) X 1/Pr(4) */
#define PLL0C_FVV 2  /* 初期値 */
#define PLL0C_MF  0  /* 初期値 */
#define PLL0C_ADJ 0  /* 初期値 */
#define PLL0C_MD  0  /* PLLモード */
#define PLL0C_SMD 0  /* 初期値 */
#define PLL0C_M   1  /* Fx:16Mhz, Mr=2 */
#define PLL0C_P   2  /* 80〜120MHz,Pr=4 */
#define PLL0C_N  59  /* Nr=60 */

#define PLL1_CLK_MHZ		80  /* PLL is 80MHz */

/* PPLLCLK(120MHz)=16MHz x (Nr(40)/Mr(2)) X 1/Pr(4) */
#define PLL1C_M   1  /* Fx:16Mhz, Mr=2 */
#define PLL1C_PA  2  /* 60〜80MHz,Pr=4 */ 
#define PLL1C_N  39  /* Nr=40 */

/*
 *  Port 10 Configration for RLIN30
 *   P10_10 : RLIN30TX
 *   P10_9  : RLIN30RX
 */
#define RLIN30_P10_MASK			((uint16) 0x0600)
#define RLIN30_PM10_INIT		((uint16) 0x0200)
#define RLIN30_PFC10_INIT		((uint16) 0x0600)
#define RLIN30_PFCE10_INIT		((uint16) 0x0000)
#define RLIN30_PFCAE10_INIT		((uint16) 0x0000)
#define RLIN30_PMC10_INIT		((uint16) 0x0600)
#define RLIN30_PIBC10_INIT		((uint16) 0x0200)

/*
 *  Port 0 Configration for RLIN31
 *   P0_5  : RLIN31TX
 *   P0_4  : RLIN31RX
 */
#define RLIN31_P0_MASK			((uint16) 0x0030)
#define RLIN31_PM0_INIT			((uint16) 0x0010)
#define RLIN31_PFC0_INIT		((uint16) 0x0000)
#define RLIN31_PFCE0_INIT		((uint16) 0x0000)
#define RLIN31_PFCAE0_INIT		((uint16) 0x0000)
#define RLIN31_PMC0_INIT		((uint16) 0x0030)
#define RLIN31_PIBC0_INIT		((uint16) 0x0010)

#endif /* TOPPERS_FL850F1L_H */
