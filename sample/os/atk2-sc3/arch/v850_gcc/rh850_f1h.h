/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
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
 *  $Id: rh850_f1h.h 187 2015-06-25 03:39:04Z t_ishikawa $
 */

/*
 *		RH850/F1Lのハードウェア資源の定義
 */

#ifndef TOPPERS_RH850_F1H_H
#define TOPPERS_RH850_F1H_H

#define _RH850G3M_

/*
 * 保護コマンドレジスタ
 */
#define PROTCMD0		0xFFF80000
#define PROTCMD1		0xFFF88000
#define CLMA0PCMD		0xFFF8C010
#define CLMA1PCMD		0xFFF8D010
#define CLMA2PCMD		0xFFF8E010
#define PROTCMDCLMA		0xFFF8C200
#define JPPCMD0			0xFFC204C0
#define PPCMD0			0xFFC14C00
#define PPCMD1			0xFFC14C04
#define PPCMD2			0xFFC14C08
#define PPCMD3			0xFFC14C0C
#define PPCMD8			0xFFC14C20
#define PPCMD9			0xFFC14C24
#define PPCMD10			0xFFC14C28
#define PPCMD11			0xFFC14C2C
#define PPCMD12			0xFFC14C30
#define PPCMD13			0xFFC14C34
#define PPCMD18			0xFFC14C48
#define PPCMD19			0xFFC14C4C
#define PPCMD20			0xFFC14C50
#define PPCMD21			0xFFC14C54
#define PPCMD22			0xFFC14C58
#define PROTCMDCVM		0xFFF83200
#define FLMDPCMD		0xFFA00004

/*
 * 保護ステータスレジスタ
 */
#define PROTS0			0xFFF80004
#define PROTS1			0xFFF88004
#define CLMA0PS			0xFFF8C014
#define CLMA1PS			0xFFF8D014
#define CLMA2PS			0xFFF8E014
#define PROTSCLMA		0xFFF8C204
#define JPPROTS0		0xFFC204B0
#define PPROTS0			0xFFC14B00
#define PPROTS1			0xFFC14B04
#define PPROTS2			0xFFC14B08
#define PPROTS3			0xFFC14B0C
#define PPROTS8			0xFFC14B20
#define PPROTS9			0xFFC14B24
#define PPROTS10		0xFFC14B28
#define PPROTS11		0xFFC14B2C
#define PPROTS12		0xFFC14B30
#define PPROTS13		0xFFC14B34
#define PPROTS18		0xFFC14B48
#define PPROTS19		0xFFC14B4C
#define PPROTS20		0xFFC14B50
#define PPROTS21		0xFFC14B54
#define PPROTS22		0xFFC14B58
#define PROTSCVM		0xFFF83204
#define FLMDPS			0xFFA00008

/*
 * 保護コマンドレジスタの番号
 */
#define PNO_CtrlProt0			0
#define PNO_CtrlProt1			1
#define PNO_ClkMonitorCtrlProt0	2
#define PNO_ClkMonitorCtrlProt1	3
#define PNO_ClkMonitorCtrlProt2	4
#define PNO_ClkMonitorTestProt	5
#define PNO_PortProt0			6
#define PNO_PortProt0_0			7
#define PNO_PortProt0_1			8
#define PNO_PortProt0_2			9
#define PNO_PortProt0_3			10
#define PNO_PortProt0_8			11
#define PNO_PortProt1_9			12
#define PNO_PortProt1_10		13
#define PNO_PortProt1_11		14
#define PNO_PortProt1_12		15
#define PNO_PortProt1_13		16
#define PNO_PortProt1_18		17
#define PNO_PortProt1_19		18
#define PNO_PortProt1_20		19
#define PNO_PortProt1_21		20
#define PNO_PortProt1_22		21
#define PNO_CoreVMonitorProt	22
#define PNO_SelfProgProt		23

/*
 *  PORTレジスタ
 */
#define PORT_BASE	UINT_C(0xffc10000)

/* 端子機能設定  (USE)*/
#define PMC(n)		((PORT_BASE) +0x0400 + (n * 0x04U))     /* ポート・モード・コントロール・レジスタ */
#define PMCSR(n)	((PORT_BASE) +0x0900 + (n * 0x04U))     /* ポート・モード・コントロール・セット／リセット・レジスタ */
#define PIPC(n)		((PORT_BASE) +0x4200 + (n * 0x04U))     /* ポートIP コントロール・レジスタ */
#define PM(n)		((PORT_BASE) +0x0300 + (n * 0x04U))     /* ポート・モード・レジスタ */
#define PMSR(n)		((PORT_BASE) +0x0800 + (n * 0x04U))     /* ポート・モード・セット／リセット・レジスタ */
#define PIBC(n)		((PORT_BASE) +0x4000 + (n * 0x04U))     /* ポート入力バッファ・コントロール・レジスタ */
#define PFC(n)		((PORT_BASE) +0x0500 + (n * 0x04U))     /* ポート機能コントロール・レジスタ */
#define PFCE(n)		((PORT_BASE) +0x0600 + (n * 0x04U))     /* ポート機能コントロール拡張・レジスタ */
#define PFCAE(n)	((PORT_BASE) +0x0A00 + (n * 0x04U))     /* ポート機能コントロール追加拡張・レジスタ */

/* 端子データ入力／出力  (USE)*/
#define PBDC(n)		((PORT_BASE) +0x4100 + (n * 0x04U))     /* ポート双方向コントロール・レジスタ */
#define PPR(n)		((PORT_BASE) +0x0200 + (n * 0x04U))     /* ポート端子リード・レジスタ */
#define P(n)		((PORT_BASE) +0x0000 + (n * 0x04U))     /* ポート・レジスタ */
#define PNOT(n)		((PORT_BASE) +0x0700 + (n * 0x04U))     /* ポート・ノット・レジスタ */
#define PSR(n)		((PORT_BASE) +0x0100 + (n * 0x04U))     /* ポート・セット／リセット・レジスタ */


#define RLN3xBASE			0xffce2040

#define RLN3xLWBR_B			0x00000001
#define RLN3xLBRP01_H		0x00000002
#define RLN3xLBRP0_B		0x00000002
#define RLN3xLBRP1_B		0x00000003
#define RLN3xLSTC_B			0x00000004
#define RLN3xLMD_B			0x00000008
#define RLN3xLBFC_B			0x00000009
#define RLN3xLSC_B			0x0000000a
#define RLN3xLWUP_B			0x0000000b
#define RLN3xLIE_B			0x0000000c
#define RLN3xLEDE_B			0x0000000d
#define RLN3xLCUC_B			0x0000000e
#define RLN3xLTRC_B			0x00000010
#define RLN3xLMST_B			0x00000011
#define RLN3xLST_B			0x00000012
#define RLN3xLEST_B			0x00000013
#define RLN3xLDFC_B			0x00000014
#define RLN3xLIDB_B			0x00000015
#define RLN3xLCBR_B			0x00000016
#define RLN3xLUDB0_B		0x00000017
#define RLN3xLDBR1_B		0x00000018
#define RLN3xLDBR2_B		0x00000019
#define RLN3xLDBR3_B		0x0000001a
#define RLN3xLDBR4_B		0x0000001b
#define RLN3xLDBR5_B		0x0000001c
#define RLN3xLDBR6_B		0x0000001d
#define RLN3xLDBR7_B		0x0000001e
#define RLN3xLDBR8_B		0x0000001f
#define RLN3xLUOER_B		0x00000020
#define RLN3xLUOR1_B		0x00000021
#define RLN3xLUTDR_H		0x00000024
#define RLN3xLUTDRL_B		0x00000024
#define RLN3xLUTDRH_B		0x00000025
#define RLN3xLURDR_H		0x00000026
#define RLN3xLURDRL_B		0x00000026
#define RLN3xLURDRH_B		0x00000027
#define RLN3xLUWTDR_H		0x00000028
#define RLN3xLUWTDRL_B		0x00000028
#define RLN3xLUWTDRH_B		0x00000029


/*
 * OSTM
 */
#if 0
#define OSTM_IRQ			UINT_C(147)

#define OSTM0_BASE	0xFFD70000
#define OSTM1_BASE	0xFFD70100
#define OSTM2_BASE	0xFFD70200
#define OSTM3_BASE	0xFFD70300
#define OSTM4_BASE	0xFFD70400
#define OSTM5_BASE	0xFFD71000
#define OSTM6_BASE	0xFFD71100
#define OSTM7_BASE	0xFFD71200
#define OSTM8_BASE	0xFFD71300
#define OSTM9_BASE	0xFFD71400

#define OSTM_CMP_W	0x00
#define OSTM_CNT_W	0x04
#define OSTM_TE		0x10
#define OSTM_TS_B	0x14
#define OSTM_TT_B	0x18
#define OSTM_CTL_B	0x20
#endif
/*
 *  PLL関連のレジスタと定義
 */
/* Main OSC */
#define MOSCE		0xfff81100
#define MOSCS		0xfff81104
#define MOSCC		0xfff81108
#define MOSCST		0xfff8110c
#define MOSCSTPM	0xfff81118

/* Sub OSC  */
#define SOSCE		0xfff81200
#define SOSCS		0xfff81204
#define SOSCST		0xfff8120C

/* PLL  */
#define PLL0E		0xfff89000
#define PLL0S		0xfff89004
#define PLL0C		0xfff89008
#define PLL1E		0xfff89100
#define PLL1S		0xfff89104
#define PLL1C		0xfff89108


#define CKSC_CPUCLKS_CTL	0xfff8a000
#define CKSC_CPUCLKS_ACT	0xfff8a008
#define CKSC_CPUCLKD_CTL	0xfff8a100
#define CKSC_CPUCLKD_ACT	0xfff8a108

#define CKSC_ILINS_CTL		0xfff8a400
#define CKSC_ILINS_ACT		0xfff8a408
#define CKSC_ILIND_CTL		0xfff8a800
#define CKSC_ILIND_ACT		0xfff8a808

#define CKSC_ATAUJS_CTL		0xfff82100
#define CKSC_ATAUJS_ACT		0xfff82108
#define CKSC_ATAUJD_CTL		0xfff82200
#define CKSC_ATAUJD_ACT		0xfff82208

#define MHz(n)		((n) * 1000 * 1000)
#define CLK_MHz(num)	(num * 1000 * 1000)

/* xxxS Register (USE) */
#define CLK_S_STPACK	0x08
#define CLK_S_CLKEN		0x04
#define CLK_S_CLKACT	0x02
#define CLK_S_CLKSTAB	0x01


/* Return Parameter */
#define UC_SUCCESS			0
#define UC_ERROR			1
#define UC_INVALIDPARAM		2
#define UC_PROTREGERROR		3
#define UC_CLKSTATUSERR		4
#define UC_CLKNOTENABLE		5
#define UC_CLKNOTACTIVE		6
#define UC_CLKNOTSTAB		7

/*
 *  RLIN3
 */
#define RLIN30_BASE 0xffce2000
#define RLIN31_BASE 0xffce2040
#define RLIN32_BASE 0xffce2080
#define RLIN33_BASE 0xffce20c0
#define RLIN34_BASE 0xffce2100
#define RLIN35_BASE 0xffce2140
  
/*
 *  INTC
 */
#define INTC1_BASE  0xFFFEEA00
#define INTC2_BASE  0xFFFFB000

#define INTC2_EIC   0x040
#define INTC2_EIBD  0x880
#define INTC2_INTNO_OFFSET 32

/* intno は unsigned を想定 */
#define EIC_ADDRESS(intno)	(intno <= 31)? (INTC1_BASE + (intno * 2)) : (INTC2_BASE + INTC2_EIC  + ((intno - INTC2_INTNO_OFFSET) * 2))

#define INTC_HAS_IBD

#define IBD_ADDRESS(intno)	(intno <= 31)? (0xFFFEEB00 + (intno * 4)) : (0xFFFFB880 + ((intno - INTC2_INTNO_OFFSET) * 4))

#define TMIN_INTNO	UINT_C(0)
#define TMAX_INTNO	UINT_C(350)
#define TNUM_INT	UINT_C(351)

/*
 *  INTNO
 */
#define RLIN30_TX_INTNO		UINT_C(34)
#define RLIN30_RX_INTNO		UINT_C(35)
#define RLIN30_ER_INTNO		UINT_C(36)
#define RLIN31_TX_INTNO		UINT_C(121)
#define RLIN31_RX_INTNO		UINT_C(122)
#define RLIN31_ER_INTNO		UINT_C(123)
#define RLIN35_TX_INTNO		UINT_C(237)
#define RLIN35_RX_INTNO		UINT_C(238)
#define RLIN35_ER_INTNO		UINT_C(239)

#define TAUFJ0I0_INTNO		UINT_C(80)
#define TAUFJ0I1_INTNO		UINT_C(81)
#define TAUFJ0I2_INTNO		UINT_C(82)
#define TAUFJ0I3_INTNO		UINT_C(83)
#define TAUFJ1I0_INTNO		UINT_C(168)
#define TAUFJ1I1_INTNO		UINT_C(169)
#define TAUFJ1I2_INTNO		UINT_C(170)
#define TAUFJ1I3_INTNO		UINT_C(171)

/*
 *  PE間割込みレジスタ
 */
#define IPIR_CH0 0xfffeec80
#define IPIR_CH1 0xfffeec84
#define IPIR_CH2 0xfffeec88
#define IPIR_CH3 0xfffeec8c

#define IPIC_ADDR(ch)  (IPIR_CH0 + ch * 4)


#ifndef TOPPERS_MACRO_ONLY

extern uint32 EnableSubOSC(void);

extern uint32 EnableMainOSC(uint32 clk_in);

extern uint32 EnablePLL0(void);

extern uint32 EnablePLL1(void);

extern uint32 SetClockSelection(uint32 s_control, uint32 s_status, uint8 regno, uint16 sel,
					uint32 d_control, uint32 d_status, uint8 divider);

extern void raise_ipir(uint8 ch);

#endif /* TOPPERS_MACRO_ONLY */


#include "v850.h"

#endif /* TOPPERS_RH850_F1H_H */
