/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2014 by FUJISOFT INCORPORATED, JAPAN
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
 *  $Id: target_config.c 42 2014-07-19 07:10:58Z ertl-honda $
 */

/*
 *		ターゲット依存モジュール（RH850F1H_PB用）
 */

#include "kernel_impl.h"
#include "v850_gcc/uart_rlin.h"
#include "v850_gcc/prc_sil.h"
#include "target_sysmod.h"
#ifdef ENABLE_RETURN_MAIN
#include "interrupt.h"
#endif /* ENABLE_RETURN_MAIN */
#ifdef TOPPERS_ENABLE_TRACE
#include "logtrace/trace_config.h"
#endif /* TOPPERS_ENABLE_TRACE */

/*
 *  文字列の出力
 */
void
target_fput_str(const char8 *c)
{
	while (*c != '\0') {
		uart_putc(*c);
		c++;
	}
	uart_putc('\n');
}


/*
 *  ポートの初期設定
 */
void
target_port_initialize(void)
{
	uint16 wk;

#ifdef RLIN3x_USE_PORT0
	/*
	 * PORT10(RLIN30)
	 */
	/* PFC10 設定 */
	wk = sil_reh_mem((void *) PFC(10));
	wk &= ~RLIN30_P10_MASK;
	wk |= (RLIN30_PFC10_INIT & RLIN30_P10_MASK);
	sil_wrh_mem((void *) PFC(10), wk);

	/* PFCE10 設定 */
	wk = sil_reh_mem((void *) PFCE(10));
	wk &= ~RLIN30_P10_MASK;
	wk |= (RLIN30_PFCE10_INIT & RLIN30_P10_MASK);
	sil_wrh_mem((void *) PFCE(10), wk);

	/* PFCAE10 設定 */
	wk = sil_reh_mem((void *) PFCAE(10));
	wk &= ~RLIN30_P10_MASK;
	wk |= (RLIN30_PFCAE10_INIT & RLIN30_P10_MASK);
	sil_wrh_mem((void *) PFCAE(10), wk);

	/* PM1 設定 */
	wk = sil_reh_mem((void *) PM(10));
	wk &= ~RLIN30_P10_MASK;
	wk |= (RLIN30_PM10_INIT & RLIN30_P10_MASK);
	sil_wrh_mem((void *) PM(10), wk);

	/* PMC10 設定 */
	wk = sil_reh_mem((void *) PMC(10));
	wk &= ~RLIN30_P10_MASK;
	wk |= (RLIN30_PMC10_INIT & RLIN30_P10_MASK);
	sil_wrh_mem((void *) PMC(10), wk);

	/* PIBC10 設定 */
	wk = sil_reh_mem((void *) PIBC(10));
	wk &= ~RLIN30_P10_MASK;
	wk |= (RLIN30_PIBC10_INIT & RLIN30_P10_MASK);
	sil_wrh_mem((void *) PIBC(10), wk);

#elif defined(RLIN3x_USE_PORT1)

	/*
	 * PORT0(RLIN31)
	 */
	/* PFC0 設定 */
	wk = sil_reh_mem((void *) PFC(0));
	wk &= ~RLIN31_P0_MASK;
	wk |= (RLIN31_PFC0_INIT & RLIN31_P0_MASK);
	sil_wrh_mem((void *) PFC(0), wk);

	/* PFCE0 設定 */
	wk = sil_reh_mem((void *) PFCE(0));
	wk &= ~RLIN31_P0_MASK;
	wk |= (RLIN31_PFCE0_INIT & RLIN31_P0_MASK);
	sil_wrh_mem((void *) PFCE(0), wk);

	/* PFCAE0 設定 */
	wk = sil_reh_mem((void *) PFCAE(0));
	wk &= ~RLIN31_P0_MASK;
	wk |= (RLIN31_PFCAE0_INIT & RLIN31_P0_MASK);
	sil_wrh_mem((void *) PFCAE(0), wk);

	/* PM1 設定 */
	wk = sil_reh_mem((void *) PM(0));
	wk &= ~RLIN31_P0_MASK;
	wk |= (RLIN31_PM0_INIT & RLIN31_P0_MASK);
	sil_wrh_mem((void *) PM(0), wk);

	/* PMC0 設定 */
	wk = sil_reh_mem((void *) PMC(0));
	wk &= ~RLIN31_P0_MASK;
	wk |= (RLIN31_PMC0_INIT & RLIN31_P0_MASK);
	sil_wrh_mem((void *) PMC(0), wk);

	/* PIBC0 設定 */
	wk = sil_reh_mem((void *) PIBC(0));
	wk &= ~RLIN31_P0_MASK;
	wk |= (RLIN31_PIBC0_INIT & RLIN31_P0_MASK);
	sil_wrh_mem((void *) PIBC(0), wk);

#else

#error define RLIN3x_USE_PORT0 or RLIN3x_USE_PORT1

#endif /* RLIN3x_USE_PORT0 */
}

/*
 *  クロック関係の初期化
 */
void
target_clock_initialize(void)
{
	uint32	errcnt = 0;

	/* Init Main Clock */
	if (EnableMainOSC(MHz(MAINOSC_CLOCK_MHZ)) != UC_SUCCESS) {
		errcnt++;
	}

	/* Init PLL0 */
	if (EnablePLL0() != UC_SUCCESS) {
		errcnt++;
	}

	/* Init PLL1 */
	if (EnablePLL1() != UC_SUCCESS) {
		errcnt++;
	}

	/* Set RLIN Clock */
	if (SetClockSelection(CKSC_ILINS_CTL, CKSC_ILINS_ACT,
					  PNO_CtrlProt1, 0x02, /* MainOSC */
					  CKSC_ILIND_CTL, CKSC_ILIND_ACT,
					  0x01) != UC_SUCCESS) {
		errcnt++;
	}

	/* Set TAUJ Clock */
	if (SetClockSelection(CKSC_ATAUJS_CTL, CKSC_ATAUJS_ACT,
						  PNO_CtrlProt0, 0x02, /* MainOSC */
						  CKSC_ATAUJD_CTL, CKSC_ATAUJD_ACT,
						  0x02 /* MainOSC/2 */
						  ) != UC_SUCCESS) {
		errcnt++;
	}

	/*
	 * Set CPU Clock
	 */
	if (SetClockSelection(CKSC_CPUCLKS_CTL, CKSC_CPUCLKS_ACT,
					  PNO_CtrlProt1, 0x03, /* 120MHz CPLLCLK */
					  CKSC_CPUCLKD_CTL, CKSC_CPUCLKD_ACT,
					  0x01) != UC_SUCCESS) {
		errcnt++;
	}

	if (errcnt > 0) {
		infinite_loop();
	}
}

extern void _reset(void);

/*
 *  ターゲット依存のハードウェアの初期化
 *  スターアップルーチンから呼び出される． 
 */
void
target_hardware_initialize(void)
{
#ifdef INIT_IBD_FOR_PE2
	/* PE2用のIBDの設定 */
	/* RLIN31 */
	sil_wrw_mem((void*)0xFFFFB9E8, 0x00010002);
	/* TAUJ1I0 */
	sil_wrw_mem((void*)0xFFFFBAA0, 0x00010002);
#endif /* INIT_IBD_FOR_PE2 */

#ifndef OMIT_CLOCK_INIT
	/* クロックの初期設定 */
	target_clock_initialize();
#else
	/* クロック初期化待ち */
	while(sil_rew_mem((void *)CKSC_CPUCLKS_CTL) != 0x03){}
#endif /* !OMIT_CLOCK_INIT */

	/* リセットベクタの初期化 */
	__LDSR(2,1,(unsigned int)_reset);

	/* ポートの初期設定 */
	target_port_initialize();
}

/*
 *  ターゲット依存の初期化
 */
void
target_initialize(void)
{
	/*
	 *  V850依存の初期化
	 */
	prc_initialize();



#ifdef TOPPERS_ENABLE_TRACE
	/*
	 *  トレースログ機能の初期化
	 */
	trace_initialize((uintptr) (TRACE_AUTOSTOP));
#endif /* TOPPERS_ENABLE_TRACE */
}

/*
 *  ターゲット依存の終了処理
 */
void
target_exit(void)
{
#ifdef TOPPERS_ENABLE_TRACE
	/*
	 *  トレースログのダンプ
	 */
	trace_dump(target_fput_log);
#endif /* TOPPERS_ENABLE_TRACE */

#ifndef ENABLE_RETURN_MAIN
	/*
	 *  シャットダウン処理の出力
	 */
	target_fput_str("Kernel Exit...");
#else
	target_fput_str("Kernel Shutdown...");
#endif /* ENABLE_RETURN_MAIN */

	/*
	 *  RH850F1H_PB依存の終了処理
	 */
	prc_terminate();

#ifdef ENABLE_RETURN_MAIN
	kerflg = FALSE;
	except_nest_cnt = 0U;
	nested_lock_os_int_cnt = 0U;
	sus_all_cnt = 0U;
	sus_all_cnt_ctx = 0U;
	sus_os_cnt = 0U;
	sus_os_cnt_ctx = 0U;

	/* スタックポインタの初期化とmain()の呼び出し */
	return_main();
#endif /* ENABLE_RETURN_MAIN */

	infinite_loop();
}

/*
 *  ターゲット依存の文字出力
 */
void
target_fput_log(char8 c)
{
	if (c == '\n') {
		uart_putc('\r');
	}
	uart_putc(c);
}
