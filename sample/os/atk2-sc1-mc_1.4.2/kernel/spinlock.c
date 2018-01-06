/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2017 by Witz Corporation
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
 *  $Id: spinlock.c 740 2017-01-25 01:19:37Z witz-itoyo $
 */

/*
 * スピンロック機能モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "mc.h"
#include "task.h"
#include "interrupt.h"
#include "spinlock.h"

/*
 *  トレースログマクロのデフォルト定義
 */

#ifndef LOG_WSUSALL_ENTER
#define LOG_WSUSALL_ENTER()
#endif /* LOG_WSUSALL_ENTER */

#ifndef LOG_WSUSALL_LEAVE
#define LOG_WSUSALL_LEAVE()
#endif /* LOG_WSUSALL_LEAVE */

#ifndef LOG_WRSMALL_ENTER
#define LOG_WRSMALL_ENTER()
#endif /* LOG_WRSMALL_ENTER */

#ifndef LOG_WRSMALL_LEAVE
#define LOG_WRSMALL_LEAVE()
#endif /* LOG_WRSMALL_LEAVE */

#ifndef LOG_WSUSOSI_ENTER
#define LOG_WSUSOSI_ENTER()
#endif /* LOG_WSUSOSI_ENTER */

#ifndef LOG_WSUSOSI_LEAVE
#define LOG_WSUSOSI_LEAVE()
#endif /* LOG_WSUSOSI_LEAVE */

#ifndef LOG_WRSMOSI_ENTER
#define LOG_WRSMOSI_ENTER()
#endif /* LOG_WRSMOSI_ENTER */

#ifndef LOG_WRSMOSI_LEAVE
#define LOG_WRSMOSI_LEAVE()
#endif /* LOG_WRSMOSI_LEAVE */

#ifndef LOG_GETSPN_ENTER
#define LOG_GETSPN_ENTER(spinlockid)
#endif /* LOG_GETSPN_ENTER */

#ifndef LOG_GETSPN_LEAVE
#define LOG_GETSPN_LEAVE(ercd)
#endif /* LOG_GETSPN_LEAVE */

#ifndef LOG_RELSPN_ENTER
#define LOG_RELSPN_ENTER(spinlockid)
#endif /* LOG_RELSPN_ENTER */

#ifndef LOG_RELSPN_LEAVE
#define LOG_RELSPN_LEAVE(ercd)
#endif /* LOG_RELSPN_LEAVE */

#ifndef LOG_TRYSPN_ENTER
#define LOG_TRYSPN_ENTER(spinlockid)
#endif /* LOG_TRYSPN_ENTER */

#ifndef LOG_TRYSPN_LEAVE
#define LOG_TRYSPN_LEAVE(ercd, p_success)
#endif /* LOG_TRYSPN_LEAVE */

#ifdef TOPPERS_spinlock_initialize

void
spinlock_initialize(void)
{
	SpinlockIdType	i;
	SPNCB			*p_spncb;
	const SPNINIB	*p_spninib;

	if (x_sense_mcore() != FALSE) {
		for (i = 1U; i <= tnum_spinlock; i++) {
			p_spninib = get_spninib(i);
			p_spncb = get_spncb(i);
			p_spncb->p_spninib = p_spninib;
			p_spncb->p_holdcb = NULL;
			p_spncb->p_ccb = NULL;
			p_spncb->p_prevspncb = NULL;
#if TTYPE_SPN == NATIVE_SPN
			x_initialize_spin(i, &(p_spncb->spn_lock));
#endif /* TTYPE_SPN == NATIVE_SPN */
		}
	}
}

#endif /* TOPPERS_spinlock_initialize */

/*
 *  ネストチェック
 */
LOCAL_INLINE boolean
check_nest_spn_order(const SPNINIB *p_get_spninib, const SPNCB *p_lastspncb)
{
	boolean			nest_flag;
	const SPNINIB	*p_spninib;

	if (p_lastspncb != NULL) {
		/* ネストしている場合 */
		nest_flag = FALSE;
		p_spninib = p_lastspncb->p_spninib;
		while (p_spninib->p_nextspn != NULL) {
			p_spninib = p_spninib->p_nextspn;
			if (p_spninib == p_get_spninib) {
				/* コンフィグレーションした順序通り */
				nest_flag = TRUE;
				break;
			}
		}
	}
	else {
		/* ネストしていない場合 */
		nest_flag = TRUE;
	}

	return(nest_flag);
}

/*
 *  スピンロックをラッピングする全割込み禁止
 */
#ifdef TOPPERS_wrap_sus_all_int

StatusType
wrap_sus_all_int(void)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_WSUSALL_ENTER();
	/* ネスト回数の上限値超過 */
	S_CHECK_LIMIT(p_ccb->sus_all_cnt != UINT8_INVALID);

	if (p_ccb->sus_all_cnt == 0U) {
		x_lock_all_int();
		ENTER_CALLEVEL(TSYS_SUSALLINT);
	}

	p_ccb->sus_all_cnt++;
	p_ccb->wrap_sus_all_cnt++;

  error_exit:
	LOG_WSUSALL_LEAVE();
	return(ercd);
}

#endif /* TOPPERS_wrap_sus_all_int */

/*
 *  スピンロックをラッピングする全割込み禁止解除
 */
#ifdef TOPPERS_wrap_res_all_int

void
wrap_res_all_int(void)
{
	CCB	*p_ccb = get_my_p_ccb();

	LOG_WRSMALL_ENTER();
	/*
	 *  内部関数なので，ここではネスト回数のチェックを行わない
	 */

	p_ccb->sus_all_cnt--;
	p_ccb->wrap_sus_all_cnt--;

	if (p_ccb->sus_all_cnt == 0U) {
		LEAVE_CALLEVEL(TSYS_SUSALLINT);
		x_unlock_all_int();
	}
	LOG_WRSMALL_LEAVE();
}

#endif /* TOPPERS_wrap_res_all_int */

/*
 *  スピンロックをラッピングするOS割込み禁止
 */
#ifdef TOPPERS_wrap_sus_os_int

StatusType
wrap_sus_os_int(void)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_WSUSOSI_ENTER();
	/* ネスト回数の上限値超過 */
	S_CHECK_LIMIT(p_ccb->sus_os_cnt != UINT8_INVALID);

	if (p_ccb->sus_os_cnt == 0U) {
		x_suspend_lock_os_int();
		ENTER_CALLEVEL(TSYS_SUSOSINT);
	}

	p_ccb->sus_os_cnt++;
	p_ccb->wrap_sus_os_cnt++;

  error_exit:
	LOG_WSUSOSI_LEAVE();
	return(ercd);
}

#endif /* TOPPERS_wrap_sus_os_int */

/*
 *  スピンロックをラッピングするOS割込み禁止解除
 */
#ifdef TOPPERS_wrap_res_os_int

void
wrap_res_os_int(void)
{
	CCB	*p_ccb = get_my_p_ccb();

	LOG_WRSMOSI_ENTER();
	/*
	 *  内部関数なので，ここではネスト回数のチェックを行わない
	 */

	p_ccb->sus_os_cnt--;
	p_ccb->wrap_sus_os_cnt--;

	if (p_ccb->sus_os_cnt == 0U) {
		LEAVE_CALLEVEL(TSYS_SUSOSINT);
		x_resume_unlock_os_int();
	}
	LOG_WRSMOSI_LEAVE();
}

#endif /* TOPPERS_wrap_res_os_int */

#if TTYPE_SPN == EMULATE_SPN
/* エミュレート方式の場合 */

/*
 *  スピンロックの取得
 */
#ifdef TOPPERS_GetSpinlock

StatusType
GetSpinlock(SpinlockIdType SpinlockId)
{
	StatusType		ercd = E_OK;
	SPNCB			*p_spncb;
	SPNCB			**pp_lastspncb;
	const SPNINIB	*p_spninib;
	CCB				*p_ccb = get_my_p_ccb();

	LOG_GETSPN_ENTER(SpinlockId);
	S_CHECK_DISALLINT();
	CHECK_CALLEVEL(CALLEVEL_GETSPINLOCK);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));
	p_spncb = get_spncb(SpinlockId);
	CHECK_INTER_DEADLOCK(p_spncb->p_ccb != p_ccb);
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		pp_lastspncb = (&(p_ccb->p_protectspncb));
	}
	else if (p_ccb->p_runisr != NULL) {
		pp_lastspncb = (&(p_ccb->p_runisr->p_lastspncb));
	}
	else {
		pp_lastspncb = (&(p_ccb->p_runtsk->p_lastspncb));
	}
	p_spninib = get_spninib(SpinlockId);
	CHECK_NEST_DEADLOCK(check_nest_spn_order(p_spninib, *pp_lastspncb) != FALSE);

	while (1) {
		/* スピンロックロックを取得する */
		x_nested_lock_os_int();
		acquire_spn_lock(GET_SPNLOCK());
		if (p_spncb->p_holdcb == NULL) {
			/* スピンロックを取得できる */
			break;
		}
		release_spn_lock(GET_SPNLOCK());
		x_nested_unlock_os_int();
		/* ここで割込み処理を実行する */
		wait_spn_lock();
	}

	/*
	 * スピンロック取得時に割込み禁止を行う場合は，ここで割込み禁止を
	 * 行う．割込み禁止がネスト回数の上限に達した場合に，ロック取得に
	 * 成功しないようにするため
	 */
	if (p_spninib->susint != NULL) {
		ercd = p_spninib->susint();
		if (ercd != E_OK) {
			release_spn_lock(GET_SPNLOCK());
			goto d_error_exit;
		}
	}

	p_spncb->p_ccb = p_ccb;
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		p_spncb->p_holdcb = (void *) &(p_ccb->p_protectspncb);
	}
	else if (p_ccb->p_runisr != NULL) {
		p_spncb->p_holdcb = (void *) (p_ccb->p_runisr);
	}
	else {
		p_spncb->p_holdcb = (void *) (p_ccb->p_runtsk);
	}
	p_spncb->p_prevspncb = *pp_lastspncb;
	*pp_lastspncb = p_spncb;

	/* スピンロックロックを解放 */
	release_spn_lock(GET_SPNLOCK());

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_GETSPN_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.spnid = SpinlockId;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetSpinlock);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetSpinlock */

/*
 *  スピンロックの解放
 */
#ifdef TOPPERS_ReleaseSpinlock

StatusType
ReleaseSpinlock(SpinlockIdType SpinlockId)
{
	StatusType		ercd = E_OK;
	SPNCB			*p_spncb;
	SPNCB			**pp_lastspncb;
	const SPNINIB	*p_spninib;
	CCB				*p_ccb = get_my_p_ccb();
	void			*p_run;

	LOG_RELSPN_ENTER(SpinlockId);
	S_CHECK_DISALLINT();
	CHECK_CALLEVEL(CALLEVEL_RELEASESPINLOCK);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));
	p_spncb = get_spncb(SpinlockId);
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		p_run = &(p_ccb->p_protectspncb);
		pp_lastspncb = &(p_ccb->p_protectspncb);
	}
	else if (p_ccb->p_runisr != NULL) {
		p_run = p_ccb->p_runisr;
		pp_lastspncb = &(p_ccb->p_runisr->p_lastspncb);
	}
	else {
		p_run = p_ccb->p_runtsk;
		pp_lastspncb = &(p_ccb->p_runtsk->p_lastspncb);
	}
	CHECK_STATE(p_run == p_spncb->p_holdcb);
	CHECK_NOFUNC(*pp_lastspncb == p_spncb);

	p_spninib = get_spninib(SpinlockId);

	/* スピンロックロックを取得する */
	x_nested_lock_os_int();
	acquire_spn_lock(GET_SPNLOCK());

	/* スピンロックを解放する */
	p_spncb->p_ccb = NULL;
	p_spncb->p_holdcb = NULL;
	*pp_lastspncb = p_spncb->p_prevspncb;

	/* スピンロックロックを解放する */
	release_spn_lock(GET_SPNLOCK());
	x_nested_unlock_os_int();

	/*
	 * スピンロック取得時に割込み禁止を行う場合は，ここで割込み禁止を
	 * 解除する
	 */
	if (p_spninib->resint != NULL) {
		/*
		 * 割込み禁止の解除でエラーが発生することはない
		 */
		p_spninib->resint();
	}

  exit_no_errorhook:
	LOG_RELSPN_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.spnid = SpinlockId;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ReleaseSpinlock);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ReleaseSpinlock */

/*
 *  スピンロック取得の試行
 */
#ifdef TOPPERS_TryToGetSpinlock

StatusType
TryToGetSpinlock(SpinlockIdType SpinlockId, TryToGetSpinlockType *Success)
{
	StatusType		ercd = E_OK;
	SPNCB			*p_spncb;
	SPNCB			**pp_lastspncb;
	const SPNINIB	*p_spninib;
	CCB				*p_ccb = get_my_p_ccb();

	LOG_TRYSPN_ENTER(SpinlockId);
	S_CHECK_DISALLINT();
	CHECK_PARAM_POINTER(Success);
	CHECK_CALLEVEL(CALLEVEL_TRYTOGETSPINLOCK);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));
	p_spncb = get_spncb(SpinlockId);
	CHECK_INTER_DEADLOCK(p_spncb->p_ccb != p_ccb);
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		pp_lastspncb = (&(p_ccb->p_protectspncb));
	}
	else if (p_ccb->p_runisr != NULL) {
		pp_lastspncb = (&(p_ccb->p_runisr->p_lastspncb));
	}
	else {
		pp_lastspncb = (&(p_ccb->p_runtsk->p_lastspncb));
	}
	p_spninib = get_spninib(SpinlockId);
	CHECK_NEST_DEADLOCK(check_nest_spn_order(p_spninib, *pp_lastspncb) != FALSE);

	*Success = TRYTOGETSPINLOCK_NOSUCCESS;

	x_nested_lock_os_int();

	if (try_spn_lock(GET_SPNLOCK()) != FALSE) {
		/* スピンロックロックが取得できた場合 */
		if (p_spncb->p_holdcb == NULL) {
			/* スピンロックが取得できる */

			/*
			 * スピンロック取得時に割込み禁止を行う場合は，ここで割込
			 * み禁止を行う．割込み禁止がネスト回数の上限に達した場合
			 * に，ロック取得に成功しないようにするため
			 */
			if (p_spninib->susint != NULL) {
				ercd = p_spninib->susint();
				if (ercd != E_OK) {
					release_spn_lock(GET_SPNLOCK());
					goto d_error_exit;
				}
			}

			p_spncb->p_ccb = p_ccb;
			if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
				p_spncb->p_holdcb = (void *) &(p_ccb->p_protectspncb);
			}
			else if (p_ccb->p_runisr != NULL) {
				p_spncb->p_holdcb = (void *) (p_ccb->p_runisr);
			}
			else {
				p_spncb->p_holdcb = (void *) (p_ccb->p_runtsk);
			}
			p_spncb->p_prevspncb = *pp_lastspncb;
			*pp_lastspncb = p_spncb;
			*Success = TRYTOGETSPINLOCK_SUCCESS;
		}
		release_spn_lock(GET_SPNLOCK());
	}

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_TRYSPN_LEAVE(ercd, Success);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.spnid = SpinlockId;
	p_ccb->temp_errorhook_par2.tryspntype = Success;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_TryToGetSpinlock);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}
#endif /* TOPPERS_TryToGetSpinlock */

/*
 *  スピンロック強制解放
 */
#ifdef TOPPERS_force_release_spinlocks

void
force_release_spinlocks(SPNCB **pp_lastspncb)
{
	FunctionRefType saved_resint;

	/* OS割込み禁止状態以上で来るはず */
	while (*pp_lastspncb != NULL) {
		acquire_spn_lock(GET_SPNLOCK());

		(*pp_lastspncb)->p_ccb = NULL;
		(*pp_lastspncb)->p_holdcb = NULL;
		saved_resint = (*pp_lastspncb)->p_spninib->resint;
		*pp_lastspncb = (*pp_lastspncb)->p_prevspncb;

		release_spn_lock(GET_SPNLOCK());

		if (saved_resint != NULL) {
			saved_resint();
		}
	} /* while (*pp_lastspncb != NULL); */
}

#endif /* TOPPERS_force_release_spinlocks */

#elif TTYPE_SPN == NATIVE_SPN
/*
 *  スピンロックの取得
 */
#ifdef TOPPERS_GetSpinlock

StatusType
GetSpinlock(SpinlockIdType SpinlockId)
{
	StatusType		ercd = E_OK;
	SPNCB			*p_spncb;
	SPNCB			**pp_lastspncb;
	const SPNINIB	*p_spninib;
	CCB				*p_ccb = get_my_p_ccb();

	LOG_GETSPN_ENTER(SpinlockId);
	S_CHECK_DISALLINT();
	CHECK_CALLEVEL(CALLEVEL_GETSPINLOCK);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));
	p_spncb = get_spncb(SpinlockId);
	CHECK_INTER_DEADLOCK(p_spncb->p_ccb != p_ccb);
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		pp_lastspncb = (&(p_ccb->p_protectspncb));
	}
	else if (p_ccb->p_runisr != NULL) {
		pp_lastspncb = (&(p_ccb->p_runisr->p_lastspncb));
	}
	else {
		pp_lastspncb = (&(p_ccb->p_runtsk->p_lastspncb));
	}
	p_spninib = get_spninib(SpinlockId);
	CHECK_NEST_DEADLOCK(check_nest_spn_order(p_spninib, *pp_lastspncb) != FALSE);

	/*
	 * CPUロック
	 * - ネイティブスピンロックの獲得と，spncbの更新を
	 *   アトミックに実施するため
	 */
	x_nested_lock_os_int();
	/*
	 * スピンロックの獲得
	 * ターゲット依存部でスピンする
	 */
	acquire_spn_lock(&(p_spncb->spn_lock));

	/*
	 *  ここでは，必ずp_spncb->p_holdcb == NULLである
	 *  acquire_spn_lock -> p_holdcb = X ->
	 *   p_holdcb = NULL -> release_spn_lock の順序で
	 *  p_holdcbの操作がされるため，p_holdcbがNULLでない
	 *  状態では，その処理単位がacquire_spn_lockしている
	 *  はずである
	 */

	/*
	 * スピンロック取得時に割込み禁止を行う場合は，ここで割込み禁止を
	 * 行う．割込み禁止がネスト回数の上限に達した場合に，ロック取得に
	 * 成功しないようにするため
	 */
	if (p_spninib->susint != NULL) {
		ercd = p_spninib->susint();
		if (ercd != E_OK) {
			release_spn_lock(&(p_spncb->spn_lock));
			goto d_error_exit;
		}
	}

	/* spncbの更新 */
	/* スピンロックを獲得したコア */
	p_spncb->p_ccb = p_ccb;
	/* スピンロックを獲得した処理単位 */
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		p_spncb->p_holdcb = (void *) &(p_ccb->p_protectspncb);
	}
	else if (p_ccb->p_runisr != NULL) {
		p_spncb->p_holdcb = (void *) (p_ccb->p_runisr);
	}
	else {
		p_spncb->p_holdcb = (void *) (p_ccb->p_runtsk);
	}
	/*
	 * 最後に獲得したスピンロックのつなぎ換え
	 * +-------+ p_lastspncb +-----------+ p_prevspncb  +-----------+
	 * | xxxcb |------------>| new spncb |------------->| old spncb |
	 * +-------+             +-----------+              +-----------+
	 */
	p_spncb->p_prevspncb = *pp_lastspncb;
	*pp_lastspncb = p_spncb;

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_GETSPN_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.spnid = SpinlockId;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetSpinlock);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetSpinlock */

/*
 *  スピンロックの解放
 */
#ifdef TOPPERS_ReleaseSpinlock

StatusType
ReleaseSpinlock(SpinlockIdType SpinlockId)
{
	StatusType		ercd = E_OK;
	SPNCB			*p_spncb;
	SPNCB			**pp_lastspncb;
	const SPNINIB	*p_spninib;
	CCB				*p_ccb = get_my_p_ccb();
	void			*p_run;

	LOG_RELSPN_ENTER(SpinlockId);
	S_CHECK_DISALLINT();
	CHECK_CALLEVEL(CALLEVEL_RELEASESPINLOCK);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));
	p_spncb = get_spncb(SpinlockId);
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		p_run = &(p_ccb->p_protectspncb);
		pp_lastspncb = &(p_ccb->p_protectspncb);
	}
	else if (p_ccb->p_runisr != NULL) {
		p_run = p_ccb->p_runisr;
		pp_lastspncb = &(p_ccb->p_runisr->p_lastspncb);
	}
	else {
		p_run = p_ccb->p_runtsk;
		pp_lastspncb = &(p_ccb->p_runtsk->p_lastspncb);
	}
	CHECK_STATE(p_run == p_spncb->p_holdcb);
	CHECK_NOFUNC(*pp_lastspncb == p_spncb);

	p_spninib = get_spninib(SpinlockId);

	/*
	 * CPUロック
	 * - ネイティブスピンロックの解放と，spncbの更新を
	 *   アトミックに実施するため
	 */
	x_nested_lock_os_int();

	/* spncbの更新 */
	p_spncb->p_ccb = NULL;
	p_spncb->p_holdcb = NULL;
	*pp_lastspncb = p_spncb->p_prevspncb;

	/* スピンロックを解放する */
	release_spn_lock(&(p_spncb->spn_lock));
	x_nested_unlock_os_int();

	/*
	 * スピンロック取得時に割込み禁止を行う場合は，ここで割込み禁止を
	 * 解除する
	 */
	if (p_spninib->resint != NULL) {
		/*
		 * 割込み禁止の解除でエラーが発生することはない
		 */
		p_spninib->resint();
	}

  exit_no_errorhook:
	LOG_RELSPN_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.spnid = SpinlockId;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ReleaseSpinlock);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ReleaseSpinlock */

/*
 *  スピンロック取得の試行
 */
#ifdef TOPPERS_TryToGetSpinlock

StatusType
TryToGetSpinlock(SpinlockIdType SpinlockId, TryToGetSpinlockType *Success)
{
	StatusType		ercd = E_OK;
	SPNCB			*p_spncb;
	SPNCB			**pp_lastspncb;
	const SPNINIB	*p_spninib;
	CCB				*p_ccb = get_my_p_ccb();

	LOG_TRYSPN_ENTER(SpinlockId);
	S_CHECK_DISALLINT();
	CHECK_PARAM_POINTER(Success);
	CHECK_CALLEVEL(CALLEVEL_TRYTOGETSPINLOCK);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));
	p_spncb = get_spncb(SpinlockId);
	CHECK_INTER_DEADLOCK(p_spncb->p_ccb != p_ccb);
	if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
		pp_lastspncb = (&(p_ccb->p_protectspncb));
	}
	else if (p_ccb->p_runisr != NULL) {
		pp_lastspncb = (&(p_ccb->p_runisr->p_lastspncb));
	}
	else {
		pp_lastspncb = (&(p_ccb->p_runtsk->p_lastspncb));
	}
	p_spninib = get_spninib(SpinlockId);
	CHECK_NEST_DEADLOCK(check_nest_spn_order(p_spninib, *pp_lastspncb) != FALSE);

	*Success = TRYTOGETSPINLOCK_NOSUCCESS;

	x_nested_lock_os_int();

	if (try_spn_lock(&(p_spncb->spn_lock)) != FALSE) {
		/* スピンロックロックが取得できた場合 */
		if (p_spncb->p_holdcb == NULL) {
			/* スピンロックが取得できる */
			/*
			 * スピンロック取得時に割込み禁止を行う場合は，ここで割込
			 * み禁止を行う．割込み禁止がネスト回数の上限に達した場合
			 * に，ロック取得に成功しないようにするため
			 */
			if (p_spninib->susint != NULL) {
				ercd = p_spninib->susint();
				if (ercd != E_OK) {
					release_spn_lock(&(p_spncb->spn_lock));
					goto d_error_exit;
				}
			}

			/* spncbの更新 */
			/* スピンロックを獲得したコア */
			p_spncb->p_ccb = p_ccb;
			/* スピンロックを獲得した処理単位 */
			if ((p_ccb->callevel_stat & TCL_PROTECT) != 0U) {
				p_spncb->p_holdcb = (void *) &(p_ccb->p_protectspncb);
			}
			else if (p_ccb->p_runisr != NULL) {
				p_spncb->p_holdcb = (void *) (p_ccb->p_runisr);
			}
			else {
				p_spncb->p_holdcb = (void *) (p_ccb->p_runtsk);
			}
			/* 最後に獲得したスピンロックのつなぎ換え */
			p_spncb->p_prevspncb = *pp_lastspncb;
			*pp_lastspncb = p_spncb;
			*Success = TRYTOGETSPINLOCK_SUCCESS;
		}
		else {
			release_spn_lock(&(p_spncb->spn_lock));
		}
	}

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_TRYSPN_LEAVE(ercd, Success);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.spnid = SpinlockId;
	p_ccb->temp_errorhook_par2.tryspntype = Success;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_TryToGetSpinlock);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}
#endif /* TOPPERS_TryToGetSpinlock */

/*
 *  スピンロック強制解放
 */
#ifdef TOPPERS_force_release_spinlocks

void
force_release_spinlocks(SPNCB **p_lastspncb)
{
	LockType		*p_release_spn_lock;
	FunctionRefType saved_resint;

	/* OS割込み禁止状態以上で来るはず */
	while (*p_lastspncb != NULL) {
		/* spncbの更新 */
		(*p_lastspncb)->p_ccb = NULL;
		(*p_lastspncb)->p_holdcb = NULL;
		saved_resint = (*p_lastspncb)->p_spninib->resint;
		p_release_spn_lock = &((*p_lastspncb)->spn_lock);
		*p_lastspncb = (*p_lastspncb)->p_prevspncb;

		/* スピンロックを解放する */
		release_spn_lock(p_release_spn_lock);

		if (saved_resint != NULL) {
			saved_resint();
		}
	} /* while (*p_lastspncb != NULL); */
}

#endif /* TOPPERS_force_release_spinlocks */

#endif /* TTYPE_SPN == NATIVE_SPN */
