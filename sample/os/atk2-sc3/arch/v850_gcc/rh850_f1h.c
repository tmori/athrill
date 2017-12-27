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
 *  $Id: rh850_f1h.c 164 2015-06-03 01:22:29Z t_ishikawa $
 */
#include "kernel_impl.h"
#include "rh850_f1h.h"
#include "Os.h"
#include "prc_sil.h"

/*******************************************************************************************/
/*  Outline      : Write protected register                                                */
/*  Argument     : Register address                                                        */
/*                 Register data                                                           */
/*                 Register No                                                             */
/*  Return value : 0: write success / 1: write error                                       */
/*  Description  : Write protected register                                                */
/*                                                                                         */
/*******************************************************************************************/
static uint32
write_protected_reg(uint32 addr, uint32 data, uint8 regno)
{
	uint8	wk;
	const uint32	cmd_reg[24] = {
		PROTCMD0,PROTCMD1,CLMA0PCMD,CLMA1PCMD,CLMA2PCMD,PROTCMDCLMA,
		JPPCMD0,PPCMD0,PPCMD1,PPCMD2,PPCMD3,PPCMD8,
		PPCMD9,PPCMD10,PPCMD11,PPCMD12,PPCMD13,PPCMD18,PPCMD19,PPCMD20,PPCMD21,PPCMD22,
		PROTCMDCVM,FLMDPCMD};
	const uint32	s_reg[24] = {
		PROTS0,PROTS1,CLMA0PS,CLMA1PS,CLMA2PS,PROTSCLMA,
		JPPROTS0,PPROTS0,PPROTS1,PPROTS2,PPROTS3,PPROTS8,
		PPROTS9,PPROTS10,PPROTS11,PPROTS12,PPROTS13,PPROTS18,PPROTS19,PPROTS20,PPROTS21,PPROTS22,
		PROTSCVM,FLMDPS};
	SIL_PRE_LOC;

	if (regno > 24) {
		return(UC_INVALIDPARAM);
	}

	SIL_LOC_INT();
	sil_wrw_mem((void *) cmd_reg[regno], 0xA5);

	sil_wrw_mem((void *) addr, data);
	sil_wrw_mem((void *) addr, ~data);
	sil_wrw_mem((void *) addr, data);
	SIL_UNL_INT();

	wk = sil_rew_mem((void *) s_reg[regno]);
	wk &= 0x01;

	return((wk == 0) ? UC_SUCCESS : UC_PROTREGERROR);
} /* write_protected_reg */


/*******************************************************************************************/
/*  Outline      : Sub Oscillator enable                                                   */
/*  Argument     : -                                                                       */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : Sub Oscillator register setting                                         */
/*                                                                                         */
/*******************************************************************************************/
uint32
EnableSubOSC(void)
{
	/* stop SubOSC */
	if (write_protected_reg(SOSCE, 0x00, PNO_CtrlProt0) != UC_SUCCESS) return(UC_ERROR);

	/* Wait inactive */
	while (((sil_rew_mem((void *) SOSCS)) & CLK_S_CLKEN) != 0) {}

	/* start SubOSC */
	if (write_protected_reg(SOSCE, 0x01, PNO_CtrlProt0) != UC_SUCCESS) return(UC_ERROR);

	/* Wait stabilization */
	while (((sil_rew_mem((void *) SOSCS)) & CLK_S_CLKEN) == 0) {}

	return(UC_SUCCESS);
} /* EnableSubOSC */

/*******************************************************************************************/
/*  Outline      : Main Oscillator enable                                                  */
/*  Argument     : Main Cscillator frequency(Hz)                                           */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : Main Oscillator register setting                                        */
/*                                                                                         */
/*******************************************************************************************/
uint32
EnableMainOSC(uint32 clk_in)
{
	uint8	ampsel;

	/* stop MainOSC */
	if (write_protected_reg(MOSCE, 0x02, PNO_CtrlProt0) != UC_SUCCESS)  return(UC_ERROR);
	
	if (clk_in == CLK_MHz(8)) {
		ampsel = 0x03;
	} else if (CLK_MHz(8) < clk_in && clk_in <= CLK_MHz(16)) {
		ampsel = 0x02;
	} else if (CLK_MHz(16) < clk_in && clk_in <= CLK_MHz(20)) {
		ampsel = 0x01;
	} else if (CLK_MHz(20) < clk_in && clk_in <= CLK_MHz(24)) {
		ampsel = 0x00;
	} else {
		return(UC_INVALIDPARAM);
	}

	/* Wait inactive */
	while (((sil_rew_mem((void *) MOSCS)) & CLK_S_CLKEN) != 0) {}

	sil_wrw_mem((void *) MOSCC, 0x00 | ampsel); /* Normal stabilization time mode */
	sil_wrw_mem((void *) MOSCST, 0x00FF);       /* stabilization time -> Max */

	/* MainOSC start */
	if (write_protected_reg(MOSCE, 0x01, PNO_CtrlProt0) != UC_SUCCESS)  return(UC_ERROR);

	/* Wait stabilization */
	while (((sil_rew_mem((void *) MOSCS)) & CLK_S_CLKEN) == 0) {}

	return(UC_SUCCESS);
} /* EnableMainOSC */

/*******************************************************************************************/
/*  Outline      : PLL0 enable                                                              */
/*  Argument     : none                                                                    */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : PLL register setting                                                    */
/*                                                                                         */
/*******************************************************************************************/
uint32
EnablePLL0(void)
{
	/* stop PLL */
	if (write_protected_reg(PLL0E, 0x02, PNO_CtrlProt1) != UC_SUCCESS)  return(UC_ERROR);

	sil_wrw_mem((void*) PLL0C,
				(PLL0C_FVV<<29) | (PLL0C_MF<<24) | (PLL0C_ADJ<<20) |
				(PLL0C_MD<<14)  | (PLL0C_SMD<<13) | (PLL0C_M<<11) |
				(PLL0C_P<<8) | (PLL0C_N));

	/* start PLL */
	if (write_protected_reg(PLL0E, 0x01, PNO_CtrlProt1) != UC_SUCCESS)  return(UC_ERROR);

	/* Wait stabilization */
	while (((sil_rew_mem((void *) PLL0S)) & CLK_S_CLKEN) == 0) {}

	return(UC_SUCCESS);
} /* EnablePLL0 */


/*******************************************************************************************/
/*  Outline      : PLL1 enable                                                              */
/*  Argument     : none                                                                    */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : PLL register setting                                                    */
/*                                                                                         */
/*******************************************************************************************/
uint32
EnablePLL1(void)
{
	/* stop PLL */
	if (write_protected_reg(PLL1E, 0x02, PNO_CtrlProt1) != UC_SUCCESS)  return(UC_ERROR);

	sil_wrw_mem((void*) PLL1C,  (PLL1C_M<<11) | (PLL1C_PA<<8) | PLL1C_N);

	/* start PLL */
	if (write_protected_reg(PLL1E, 0x01, PNO_CtrlProt1) != UC_SUCCESS)  return(UC_ERROR);

	/* Wait stabilization */
	while (((sil_rew_mem((void *) PLL1S)) & CLK_S_CLKEN) == 0) {}

	return(UC_SUCCESS);
} /* EnablePLL1 */

/*******************************************************************************************/
/*  Outline      : Clock switcher setting                                                  */
/*  Argument     : Selector Control reginster address                                      */
/*                 Selector Status reginster address                                       */
/*                 Register No                                                             */
/*                 Select No                                                               */
/*                 Divider Control reginster address                                       */
/*                 Divider Status reginster address                                        */
/*                 divider                                                                 */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : Select clock source of CKSCLK_mn                                        */
/*                                                                                         */
/*******************************************************************************************/
uint32
SetClockSelection(uint32 s_control, uint32 s_status, uint8 regno, uint16 sel,
					uint32 d_control, uint32 d_status, uint8 divider)
{
	/* Set Selector */
	if (write_protected_reg(s_control, sel, regno) != UC_SUCCESS) { return(UC_ERROR);}

	/* Set Divider */
	if (d_control != 0) {
		if (write_protected_reg(d_control, divider, regno) != UC_SUCCESS) { return(UC_ERROR);}
	}

	/* Wait untile enable */
	while (sil_rew_mem((void *) s_control) != sil_rew_mem((void *) s_status)) {};
	if (d_control != 0) {
		while (sil_rew_mem((void *) d_control) != sil_rew_mem((void *) d_status)) {};
	}

	return(UC_SUCCESS);
}

void
raise_ipir(uint8 ch) {
	if (current_cpuid() == 1) {
		sil_wrw_mem((void*)IPIC_ADDR(ch), 2);
	}
	else {
		sil_wrw_mem((void*)IPIC_ADDR(ch), 1);
	}
}
