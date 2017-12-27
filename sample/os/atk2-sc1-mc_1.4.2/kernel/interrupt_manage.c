/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
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
 *  $Id: interrupt_manage.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		割込み管理モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "interrupt.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_DISINT_ENTER
#define LOG_DISINT_ENTER()
#endif /* LOG_DISINT_ENTER */

#ifndef LOG_DISINT_LEAVE
#define LOG_DISINT_LEAVE()
#endif /* LOG_DISINT_LEAVE */

#ifndef LOG_ENAINT_ENTER
#define LOG_ENAINT_ENTER()
#endif /* LOG_ENAINT_ENTER */

#ifndef LOG_ENAINT_LEAVE
#define LOG_ENAINT_LEAVE()
#endif /* LOG_ENAINT_LEAVE */

#ifndef LOG_SUSALL_ENTER
#define LOG_SUSALL_ENTER()
#endif /* LOG_SUSALL_ENTER */

#ifndef LOG_SUSALL_LEAVE
#define LOG_SUSALL_LEAVE()
#endif /* LOG_SUSALL_LEAVE */

#ifndef LOG_RSMALL_ENTER
#define LOG_RSMALL_ENTER()
#endif /* LOG_RSMALL_ENTER */

#ifndef LOG_RSMALL_LEAVE
#define LOG_RSMALL_LEAVE()
#endif /* LOG_RSMALL_LEAVE */

#ifndef LOG_SUSOSI_ENTER
#define LOG_SUSOSI_ENTER()
#endif /* LOG_SUSOSI_ENTER */

#ifndef LOG_SUSOSI_LEAVE
#define LOG_SUSOSI_LEAVE()
#endif /* LOG_SUSOSI_LEAVE */

#ifndef LOG_RSMOSI_ENTER
#define LOG_RSMOSI_ENTER()
#endif /* LOG_RSMOSI_ENTER */

#ifndef LOG_RSMOSI_LEAVE
#define LOG_RSMOSI_LEAVE()
#endif /* LOG_RSMOSI_LEAVE */

#ifndef LOG_GETISRID_ENTER
#define LOG_GETISRID_ENTER()
#endif /* LOG_GETISRID_ENTER */

#ifndef LOG_GETISRID_LEAVE
#define LOG_GETISRID_LEAVE(ercd)
#endif /* LOG_GETISRID_LEAVE */

#ifndef LOG_DISINTSRC_ENTER
#define LOG_DISINTSRC_ENTER(isrid)
#endif /* LOG_DISINTSRC_ENTER */

#ifndef LOG_DISINTSRC_LEAVE
#define	LOG_DISINTSRC_LEAVE(ercd)
#endif /* LOG_DISINTSRC_LEAVE */

#ifndef LOG_ENAINTSRC_ENTER
#define LOG_ENAINTSRC_ENTER(isrid)
#endif /* LOG_ENAINTSRC_ENTER */

#ifndef LOG_ENAINTSRC_LEAVE
#define	LOG_ENAINTSRC_LEAVE(ercd)
#endif /* LOG_ENAINTSRC_LEAVE */

/*
 *  すべての割込みの禁止（高速簡易版）
 *  全割込み禁止状態へ移行
 */
#ifdef TOPPERS_DisableAllInterrupts

#ifndef OMIT_STANDARD_DISALLINT

void
DisableAllInterrupts(void)
{
	LOG_DISINT_ENTER();
	if ((get_my_p_ccb()->callevel_stat & (TSYS_DISALLINT | TSYS_SUSALLINT | TSYS_SUSOSINT)) == TSYS_NULL) {
		x_lock_all_int();
		ENTER_CALLEVEL(TSYS_DISALLINT);
	}
	LOG_DISINT_LEAVE();
}

#endif /* OMIT_STANDARD_DISALLINT */

#endif /* TOPPERS_DisableAllInterrupts */

/*
 *  すべての割込みの許可（高速簡易版）
 *  全割込み禁止状態を解除する
 */
#ifdef TOPPERS_EnableAllInterrupts

#ifndef OMIT_STANDARD_DISALLINT

void
EnableAllInterrupts(void)
{
	CCB *p_ccb = get_my_p_ccb();

	LOG_ENAINT_ENTER();
	if ((p_ccb->callevel_stat & (TSYS_SUSALLINT | TSYS_SUSOSINT)) == TSYS_NULL) {
		if ((p_ccb->callevel_stat & TSYS_DISALLINT) != TSYS_NULL) {
			LEAVE_CALLEVEL(TSYS_DISALLINT);
			x_unlock_all_int();
		}
	}
	LOG_ENAINT_LEAVE();
}

#endif /* OMIT_STANDARD_DISALLINT */

#endif /* TOPPERS_EnableAllInterrupts */

/*
 *  全割込み禁止
 *  CPU全ての割込みが対象の割込み禁止(ネストカウント有り)
 */
#ifdef TOPPERS_SuspendAllInterrupts

void
SuspendAllInterrupts(void)
{
#ifdef CFG_USE_ERRORHOOK
	StatusType	ercd;
#endif /* CFG_USE_ERRORHOOK */
	CCB			*p_ccb = get_my_p_ccb();

	LOG_SUSALL_ENTER();
	S_N_CHECK_DISALLINT();
	/* ネスト回数の上限値超過 */
	S_N_CHECK_LIMIT(p_ccb->sus_all_cnt != UINT8_INVALID);

	if (p_ccb->sus_all_cnt == 0U) {
		x_lock_all_int();
		ENTER_CALLEVEL(TSYS_SUSALLINT);
	}

	p_ccb->sus_all_cnt++;

  exit_no_errorhook:
	LOG_SUSALL_LEAVE();
	return;

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	if (p_ccb->kerflg != FALSE) {
		x_nested_lock_os_int();
		call_errorhook(ercd, OSServiceId_SuspendAllInterrupts);
		x_nested_unlock_os_int();
		goto exit_no_errorhook;
	}
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_SuspendAllInterrupts */

/*
 *  全割込み禁止解除
 *  CPU全ての割込みが対象の割込み許可(ネストカウント有り)
 */
#ifdef TOPPERS_ResumeAllInterrupts

void
ResumeAllInterrupts(void)
{
#ifdef CFG_USE_ERRORHOOK
	StatusType	ercd;
#endif /* CFG_USE_ERRORHOOK */
	CCB			*p_ccb = get_my_p_ccb();

	LOG_RSMALL_ENTER();
	S_N_CHECK_DISALLINT();
	S_N_CHECK_STATE(p_ccb->sus_all_cnt != 0U);
	S_N_CHECK_STATE(p_ccb->sus_all_cnt > p_ccb->wrap_sus_all_cnt);

	p_ccb->sus_all_cnt--;

	if (p_ccb->sus_all_cnt == 0U) {
		LEAVE_CALLEVEL(TSYS_SUSALLINT);
		x_unlock_all_int();
	}

  exit_no_errorhook:
	LOG_RSMALL_LEAVE();
	return;

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	if (p_ccb->kerflg != FALSE) {
		x_nested_lock_os_int();
		call_errorhook(ercd, OSServiceId_ResumeAllInterrupts);
		x_nested_unlock_os_int();
		goto exit_no_errorhook;
	}
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ResumeAllInterrupts */

/*
 *  OS割込みの禁止
 *  C2ISRが対象の割込み禁止(ネストカウント有り)
 */
#ifdef TOPPERS_SuspendOSInterrupts

void
SuspendOSInterrupts(void)
{
#ifdef CFG_USE_ERRORHOOK
	StatusType	ercd;
#endif /* CFG_USE_ERRORHOOK */
	CCB			*p_ccb = get_my_p_ccb();

	LOG_SUSOSI_ENTER();
	if ((p_ccb->callevel_stat & (TCL_PROTECT | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_ERROR | TCL_ALRMCBAK | TSYS_ISR1)) == TCL_NULL) {
		S_N_CHECK_DISALLINT();
		/* ネスト回数の上限値超過 */
		S_N_CHECK_LIMIT(p_ccb->sus_os_cnt != UINT8_INVALID);

		if (p_ccb->sus_os_cnt == 0U) {
			x_nested_lock_os_int();
			ENTER_CALLEVEL(TSYS_SUSOSINT);
		}

		p_ccb->sus_os_cnt++;
	}

  exit_no_errorhook:
	LOG_SUSOSI_LEAVE();
	return;

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_SuspendOSInterrupts);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_SuspendOSInterrupts */

/*
 *  OS割込み禁止解除
 *  C2ISRが対象の割込み許可(ネストカウント有り)
 */
#ifdef TOPPERS_ResumeOSInterrupts

void
ResumeOSInterrupts(void)
{
#ifdef CFG_USE_ERRORHOOK
	StatusType	ercd;
#endif /* CFG_USE_ERRORHOOK */
	CCB			*p_ccb = get_my_p_ccb();

	LOG_RSMOSI_ENTER();
	if ((p_ccb->callevel_stat & (TCL_PROTECT | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN | TCL_ERROR | TCL_ALRMCBAK | TSYS_ISR1)) == TCL_NULL) {
		S_N_CHECK_DISALLINT();
		S_N_CHECK_STATE(p_ccb->sus_os_cnt != 0U);
		S_N_CHECK_STATE(p_ccb->sus_os_cnt > p_ccb->wrap_sus_os_cnt);

		p_ccb->sus_os_cnt--;

		if (p_ccb->sus_os_cnt == 0U) {
			LEAVE_CALLEVEL(TSYS_SUSOSINT);
			x_nested_unlock_os_int();
		}
	}

  exit_no_errorhook:
	LOG_RSMOSI_LEAVE();
	return;

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_ResumeOSInterrupts);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ResumeOSInterrupts */

/*
 *  C2ISR IDの取得
 */
#ifdef TOPPERS_GetISRID

ISRType
GetISRID(void)
{
	ISRType		isrid;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType	ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	CCB			*p_ccb = get_my_p_ccb();

	LOG_GETISRID_ENTER();
	CHECK_CALLEVEL(CALLEVEL_GETISRID);

	isrid = (p_ccb->p_runisr == NULL) ? INVALID_ISR : ISR2ID(p_ccb->p_runisr);

  exit_finish:
	LOG_GETISRID_LEAVE(isrid);
	return(isrid);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	/*
	 *  エラー発生時はINVALID_ISRIDが返るが，エラーが発生したのか実行中の
	 *  C2ISRが存在しないのか区別するため，エラーフックを呼ぶ
	 */
	call_errorhook(ercd, OSServiceId_GetISRID);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	isrid = INVALID_ISR;
	goto exit_finish;
}

#endif /* TOPPERS_GetISRID */

/*
 *  割込みの許可
 */
#ifdef TOPPERS_EnableInterruptSource

StatusType
DisableInterruptSource(ISRType DisableISR)
{
	ISRCB		*p_isrcb;
	StatusType	ercd;
	CCB			*my_p_ccb;

	LOG_DISINTSRC_ENTER(DisableISR);
	CHECK_CALLEVEL(CALLEVEL_DISABLEINTERRUPTSOURCE);
	CHECK_ID(DisableISR < tnum_isr2);
	p_isrcb = get_isrcb(DisableISR);
	CHECK_NOFUNC(target_is_int_controllable(p_isrcb->p_isrinib->p_intinib->intno) != FALSE);
	CHECK_CORE(p_isrcb->p_isrinib != NULL);
	CHECK_CORE(GET_ISR_COREID(p_isrcb) == x_core_id());

	if (DisableISR < (tnum_isr2 - tnum_ici)) {
		/* C2ISR */
		x_disable_int(GET_INTNO(p_isrcb));
	}
	else {
		/* ICISR */
		my_p_ccb = get_my_p_ccb();
		x_nested_lock_os_int();
		acquire_tsk_lock(my_p_ccb);

		/* ICISR対応したビットをクリアして，コア間割込みを禁止する */
		my_p_ccb->ici_bit_mask &= ~(iciinib_table[(DisableISR + tnum_ici) - tnum_isr2].ici_bit_ptn);

		release_tsk_lock(my_p_ccb);
		x_nested_unlock_os_int();
	}
	ercd = E_OK;

  exit_no_errorhook:
	LOG_DISINTSRC_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.isrid = DisableISR;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_DisableInterruptSource);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_DisableInterruptSource */

/*
 *  割込みの許可
 */
#ifdef TOPPERS_EnableInterruptSource

StatusType
EnableInterruptSource(ISRType EnableISR)
{
	ISRCB		*p_isrcb;
	StatusType	ercd;
	CCB			*my_p_ccb;

	LOG_ENAINTSRC_ENTER(EnableISR);
	CHECK_CALLEVEL(CALLEVEL_ENABLEINTERRUPTSOURCE);
	CHECK_ID(EnableISR < tnum_isr2);
	p_isrcb = get_isrcb(EnableISR);
	CHECK_NOFUNC(target_is_int_controllable(p_isrcb->p_isrinib->p_intinib->intno) != FALSE);
	CHECK_CORE(p_isrcb->p_isrinib != NULL);
	CHECK_CORE(GET_ISR_COREID(p_isrcb) == x_core_id());

	if (EnableISR < (tnum_isr2 - tnum_ici)) {
		/* C2ISR */
		x_enable_int(GET_INTNO(p_isrcb));
	}
	else {
		/* ICISR */
		my_p_ccb = get_my_p_ccb();
		x_nested_lock_os_int();
		acquire_tsk_lock(my_p_ccb);

		/* ICISR対応したビットをセットして，コア間割込みを許可する */
		my_p_ccb->ici_bit_mask |= iciinib_table[(EnableISR + tnum_ici) - tnum_isr2].ici_bit_ptn;

		/* 当該コア割込みビットがセットされた場合，自コアに対してコア間割込みを発行する */
		if ((my_p_ccb->ici_request_map & iciinib_table[(EnableISR + tnum_ici) - tnum_isr2].ici_bit_ptn) != 0U) {
			target_ici_raise(my_p_ccb->coreid);
		}
		release_tsk_lock(my_p_ccb);
		x_nested_unlock_os_int();
	}
	ercd = E_OK;

  exit_no_errorhook:
	LOG_ENAINTSRC_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.isrid = EnableISR;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_EnableInterruptSource);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_EnableInterruptSource */
