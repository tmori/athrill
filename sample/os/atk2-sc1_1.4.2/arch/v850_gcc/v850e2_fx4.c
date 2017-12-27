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
 *  $Id: v850e2_fx4.c 540 2015-12-29 01:00:01Z ertl-honda $
 */
#include "kernel_impl.h"
#include "v850e2_fx4.h"
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
	uint32	reg_top = 0xff420000;
	uint32	reg_stat;
	uint8	wk;
	SIL_PRE_LOC;

	if (regno > 2) {
		return(UC_INVALIDPARAM);
	}

	switch (regno) {
	case  0:
		reg_top += 0x4000;      /* PROTCMD0 */
		break;
	case  1:
		reg_top += 0x8000;      /* PROTCMD1 */
		break;
	case  2:
		reg_top += 0x0300;      /* PROTCMD2 */
		break;
	}
	reg_stat = reg_top + 4;     /* PROTS0/PROTS1/PROTS2 */

	SIL_LOC_INT();
	sil_wrb_mem((void *) reg_top, 0xA5);

	sil_wrw_mem((void *) addr, data);
	sil_wrw_mem((void *) addr, ~data);
	sil_wrw_mem((void *) addr, data);
	SIL_UNL_INT();

	wk = sil_reb_mem((void *) reg_stat);
	wk &= 0x01;

	return((wk == 0) ? UC_SUCCESS : UC_PROTREGERROR);
} /* write_protected_reg */

/********************************************************************************************/
/*  Function Name : V850Drv_nop                                                             */
/*  Input         : none                                                                    */
/*  Output        : none                                                                    */
/*  Description   : nop command                                                             */
/********************************************************************************************/
static void
V850Drv_nop(void)
{
	Asm("nop");

} /* V850Drv_nop */

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
	uint32 ucret;

	sil_wrw_mem((void *) SOSCST, 0x02);      /* stabilization time -> 262ms */

	ucret = write_protected_reg(SOSCE, 0x01, PROT_SOSCE);  /* SubOSC start */

	if (ucret != UC_SUCCESS) return(ucret);

	while (((sil_rew_mem((void *) SOSCS)) & CLK_S_CLKSTAB) == 0) {     /* Wait stabilization */
		V850Drv_nop();
	}
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
	uint32	ucret;

	if (clk_in == CLK_MHz(4)) {
		ampsel = 0x03;
	}
	else if (CLK_MHz(4) < clk_in && clk_in <= CLK_MHz(8)) {
		ampsel = 0x02;
	}
	else if (CLK_MHz(8) < clk_in && clk_in <= CLK_MHz(16)) {
		ampsel = 0x01;
	}
	else if (CLK_MHz(16) < clk_in && clk_in <= CLK_MHz(20)) {
		ampsel = 0x00;
	}
	else {
		return(UC_INVALIDPARAM);
	}

	sil_wrw_mem((void *) MOSCC, 0x00 | ampsel); /* Normal stabilization time mode */
	sil_wrw_mem((void *) MOSCST, 0x0F);         /* stabilization time -> Max */

	ucret = write_protected_reg(MOSCE, 0x01, PROT_MOSCE);   /* MainOSC start */

	if (ucret != UC_SUCCESS) return(ucret);

	while (((sil_rew_mem((void *) MOSCS)) & CLK_S_CLKSTAB) == 0) {     /* Wait stabilization */
		V850Drv_nop();
	}

	return(UC_SUCCESS);
} /* EnableMainOSC */

/*******************************************************************************************/
/*  Outline      : PLL enable                                                              */
/*  Argument     : PLL No                                                                  */
/*                 Multiplying rate                                                        */
/*                 Register data                                                           */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : PLL register setting                                                    */
/*                                                                                         */
/*******************************************************************************************/
static uint32
EnablePLL(uint32 pllno, uint8 clk_mul, uint8 p_val)
{
	uint32 ucret;

	if (clk_mul > 50)
		return(UC_INVALIDPARAM);

	sil_wrw_mem((void *) PLLC(pllno), (0x800000 | (p_val << 8) | (clk_mul - 1)));   /* PLL Mode, P, Nr */
	sil_wrw_mem((void *) PLLST(pllno),  0x07);                                      /* stabilization time -> Max */

	ucret = write_protected_reg(PLLE(pllno), 0x01, PROT_PLLE);  /* PLL Start */

	if (ucret != UC_SUCCESS)
		return(ucret);

	while (((sil_rew_mem((void *) PLLS(pllno))) & CLK_S_CLKSTAB) == 0) {     /* Wait stabilization */
		/* V850Drv_nop(); */
	}

	return(UC_SUCCESS);
} /* EnablePLL */

/*******************************************************************************************/
/*  Outline      : Set PLL frequency                                                       */
/*  Argument     : PLL select No                                                           */
/*                 PLL frequency                                                           */
/*                 PLL frequency                                                           */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : Set PLL register from frequency                                         */
/*                                                                                         */
/*******************************************************************************************/
uint32
SetPLL(uint32 pllno, uint32 mhz, uint32 *outclk)
{
	uint32 mul;

	if (mhz < 20) {
		return(UC_ERROR);
	}
	else if (mhz < 50) {
		mul = (mhz / MAINOSC_CLOCK) * 4;
		if (mul <= 5 && 51 <= mul)
			return(UC_ERROR);
		*outclk = MAINOSC_CLOCK * mul / 4;
		return(EnablePLL(pllno, mul, PDIV4R0_025TO050));
	}
	else if (mhz < 100) {
		mul = (mhz / MAINOSC_CLOCK) * 2;
		if (mul <= 5 && 51 <= mul)
			return(UC_ERROR);
		*outclk = MAINOSC_CLOCK * mul / 2;
		return(EnablePLL(pllno, mul, PDIV2R0_050TO100));
	}
	else if (mhz < 200) {
		mul = (mhz / MAINOSC_CLOCK) * 1;
		if (mul <= 5 && 51 <= mul)
			return(UC_ERROR);
		*outclk = MAINOSC_CLOCK * mul / 1;
		return(EnablePLL(pllno, mul, PDIV1R0_100TO200));
	}
	else if (mhz < 400) {
		mul = (mhz / MAINOSC_CLOCK) / 2;
		if (mul <= 5 && 51 <= mul)
			return(UC_ERROR);
		*outclk = MAINOSC_CLOCK * mul * 2;
		return(EnablePLL(pllno, mul, PDIV0R5_200TO400));
	}
	return(UC_ERROR);
} /* SetPLL */

/*******************************************************************************************/
/*  Outline      : Clock switcher setting                                                  */
/*  Argument     : Control reginster address                                               */
/*                 Status reginster address                                                */
/*                 Register No                                                             */
/*                 Select No                                                               */
/*  Return value : 0: successfly set / 1: set error                                        */
/*  Description  : Select clock source of CKSCLK_mn                                        */
/*                                                                                         */
/*******************************************************************************************/
uint32
set_clock_selection(uint32 control, uint32 status, uint8 regno, uint16 sel)
{
	uint32 ucret;

	ucret = write_protected_reg(control, sel << 1, regno);
	if (ucret != UC_SUCCESS) {
		return(ucret);
	}

	/* Wait for SelectEnable */
	while (((sil_rew_mem((void *) status)) & 0x01) == 0) {
		V850Drv_nop();
	}

	return(UC_SUCCESS);
} /* set_clock_selection */
