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
 *  $Id: task_manage.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		タスク管理モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_TSKSTAT
#define LOG_TSKSTAT(p_tcb)
#endif /* LOG_TSKSTAT */

#ifndef LOG_ACTTSK_ENTER
#define LOG_ACTTSK_ENTER(tskid)
#endif /* LOG_ACTTSK_ENTER */

#ifndef LOG_ACTTSK_LEAVE
#define LOG_ACTTSK_LEAVE(ercd)
#endif /* LOG_ACTTSK_LEAVE */

#ifndef LOG_TERTSK_ENTER
#define LOG_TERTSK_ENTER()
#endif /* LOG_TERTSK_ENTER */

#ifndef LOG_TERTSK_LEAVE
#define LOG_TERTSK_LEAVE(ercd)
#endif /* LOG_TERTSK_LEAVE */

#ifndef LOG_CHNTSK_ENTER
#define LOG_CHNTSK_ENTER(tskid)
#endif /* LOG_CHNTSK_ENTER */

#ifndef LOG_CHNTSK_LEAVE
#define LOG_CHNTSK_LEAVE(ercd)
#endif /* LOG_CHNTSK_LEAVE */

#ifndef LOG_SCHED_LEAVE
#define LOG_SCHED_ENTER()
#endif /* LOG_SCHED_LEAVE */

#ifndef LOG_SCHED_LEAVE
#define LOG_SCHED_LEAVE(ercd)
#endif /* LOG_SCHED_LEAVE */

#ifndef LOG_GETTID_ENTER
#define LOG_GETTID_ENTER()
#endif /* LOG_GETTID_ENTER */

#ifndef LOG_GETTID_LEAVE
#define LOG_GETTID_LEAVE(ercd, p_tskid)
#endif /* LOG_GETTID_LEAVE */

#ifndef LOG_GETTST_ENTER
#define LOG_GETTST_ENTER(tskid)
#endif /* LOG_GETTST_ENTER */

#ifndef LOG_GETTST_LEAVE
#define LOG_GETTST_LEAVE(ercd, p_state)
#endif /* LOG_GETTST_LEAVE */


/*
 *  タスクの起動
 */
#ifdef TOPPERS_ActivateTask

StatusType
ActivateTask(TaskType TaskID)
{
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	boolean		dspreq = FALSE;
	CCB			*p_ccb;

	LOG_ACTTSK_ENTER(TaskID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_ACTIVATETASK);
	CHECK_ID(TaskID < tnum_task);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	p_ccb = p_tcb->p_tinib->p_ccb;

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);

	if (p_tcb->tstat == SUSPENDED) {
		if (make_active(p_tcb) != FALSE) {
			/*
			 *  タスクtskidが他コアであればディスパッチ要求
			 *  自コアでかつタスクコンテキストから呼び出されていた場合dspreqを真に
			 */
			if ((dispatch_request(p_ccb) != FALSE) && (p_ccb->callevel_stat == TCL_TASK)) {
				dspreq = TRUE;
			}
		}
	}
	else {
		S_D_CHECK_LIMIT(p_tcb->actcnt < p_tcb->p_tinib->maxact);

		p_tcb->actcnt += 1U;
	}

	release_tsk_lock_and_dispatch(p_ccb, dspreq);
	x_nested_unlock_os_int();

  exit_no_errorhook:
	LOG_ACTTSK_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	release_tsk_lock(p_ccb);
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ActivateTask);
#else /* CFG_USE_ERRORHOOK */
  d_exit_no_errorhook:
	release_tsk_lock(p_ccb);
#endif /* CFG_USE_ERRORHOOK */
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
}

#endif /* TOPPERS_ActivateTask */

/*
 *  自タスクの終了
 */
#ifdef TOPPERS_TerminateTask

StatusType
TerminateTask(void)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_TERTSK_ENTER();
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_TERMINATETASK);
	CHECK_RESOURCE(p_ccb->p_runtsk->p_lastrescb == NULL);
	CHECK_SPINLOCK(p_ccb->p_runtsk->p_lastspncb == NULL);

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);
	/*
	 *  内部リソースの解放は優先度を下げるだけなので，ここでは
	 *  何もしなくてよい
	 */

	suspend(p_ccb);

	release_tsk_lock(p_ccb);
	LOG_TERTSK_LEAVE(E_OK);
	exit_and_dispatch();
	ASSERT_NO_REACHED;

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_TerminateTask);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	LOG_TERTSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_TerminateTask */

/*
 *  自タスクの終了とタスクの起動
 */
#ifdef TOPPERS_ChainTask

StatusType
ChainTask(TaskType TaskID)
{
	/*
	 *  ここでの ercd の初期化は本来は不要であるが，コンパイラの警
	 *  告メッセージを避けるために初期化している
	 */
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	CCB			*my_p_ccb;
	CCB			*t_p_ccb;
	CCB			*lock_p_ccb;

	LOG_CHNTSK_ENTER(TaskID);
	my_p_ccb = get_my_p_ccb();

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHAINTASK);
	CHECK_RESOURCE(my_p_ccb->p_runtsk->p_lastrescb == NULL);
	CHECK_SPINLOCK(my_p_ccb->p_runtsk->p_lastspncb == NULL);
	CHECK_ID(TaskID < tnum_task);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	t_p_ccb = p_tcb->p_tinib->p_ccb;

	if (my_p_ccb == t_p_ccb) {
		/*
		 * 終了タスクと後続タスクが同じコアに割り付けられている場合
		 */
		lock_p_ccb = my_p_ccb;
		x_nested_lock_os_int();
		acquire_tsk_lock(lock_p_ccb);

		if (p_tcb == my_p_ccb->p_runtsk) {
			make_non_runnable(my_p_ccb);
			(void) make_active(my_p_ccb->p_runtsk);
		}
		else {
			/*
			 *  エラー時に副作用が残らないように，エラーチェックは
			 *  タスク終了処理の前に行う必要がある
			 */
			S_D_CHECK_LIMIT((p_tcb->tstat == SUSPENDED)
							|| (p_tcb->actcnt < p_tcb->p_tinib->maxact));

			suspend(my_p_ccb);

			if (p_tcb->tstat == SUSPENDED) {
				(void) make_active(p_tcb);
			}
			else {
				p_tcb->actcnt += 1U;
			}
		}
	}
	else {
		/*
		 * 終了タスクと後続タスクが異なるコアに割り付けられている場合
		 */
		lock_p_ccb = t_p_ccb;
		x_nested_lock_os_int();
		acquire_tsk_lock(lock_p_ccb);

		/*
		 *  エラー時に副作用が残らないように，エラーチェックは
		 *  タスク終了処理の前に行う必要がある
		 */
		S_D_CHECK_LIMIT((p_tcb->tstat == SUSPENDED)
						|| (p_tcb->actcnt < p_tcb->p_tinib->maxact));
		if (p_tcb->tstat == SUSPENDED) {
			if (make_active(p_tcb) != FALSE) {
				/* ディスパッチ要求 */
				(void) dispatch_request(t_p_ccb);
			}
		}
		else {
			p_tcb->actcnt += 1U;
		}
		release_tsk_lock(lock_p_ccb);
		x_nested_unlock_os_int();

		/* 自タスク終了 */
		x_nested_lock_os_int();
		acquire_tsk_lock(my_p_ccb);
		suspend(my_p_ccb);
		lock_p_ccb = my_p_ccb;
	}
	release_tsk_lock(lock_p_ccb);
	LOG_CHNTSK_LEAVE(E_OK);
	exit_and_dispatch();
	ASSERT_NO_REACHED;

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	release_tsk_lock(lock_p_ccb);
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
#ifdef CFG_USE_PARAMETERACCESS
	my_p_ccb->temp_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ChainTask);
#else /* CFG_USE_ERRORHOOK */
  d_exit_no_errorhook:
	release_tsk_lock(lock_p_ccb);
#endif /* CFG_USE_ERRORHOOK */
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_CHNTSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ChainTask */

/*
 *  スケジューラの呼び出し
 */
#ifdef TOPPERS_Schedule

StatusType
Schedule(void)
{
	/*
	 *  ここでの ercd の初期化は本来は不要であるが，コンパイラの警
	 *  告メッセージを避けるために初期化している
	 */
	StatusType		ercd = E_OK;
	CCB				*p_ccb = get_my_p_ccb();
	PriorityType	pre_pri;

	LOG_SCHED_ENTER();
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_SCHEDULE);
	CHECK_RESOURCE(p_ccb->p_runtsk->p_lastrescb == NULL);
	CHECK_SPINLOCK(p_ccb->p_runtsk->p_lastspncb == NULL);

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);
	if (p_ccb->p_runtsk->p_tinib->inipri > p_ccb->nextpri) {
		pre_pri = p_ccb->p_runtsk->curpri;
		p_ccb->p_runtsk->curpri = p_ccb->p_runtsk->p_tinib->inipri;
		preempt(p_ccb, pre_pri);
		release_tsk_lock(p_ccb);
		dispatch();
		p_ccb->p_runtsk->curpri = p_ccb->p_runtsk->p_tinib->exepri;
	}
	else {
		release_tsk_lock(p_ccb);
	}

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_SCHED_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_Schedule);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_Schedule */

/*
 *  実行状態のタスクIDの参照
 */
#ifdef TOPPERS_GetTaskID

StatusType
GetTaskID(TaskRefType TaskID)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_GETTID_ENTER();
	CHECK_CALLEVEL(CALLEVEL_GETTASKID);
	CHECK_PARAM_POINTER(TaskID);
	*TaskID = (p_ccb->p_runtsk == NULL) ? INVALID_TASK : TSKID(p_ccb->p_runtsk);

  exit_no_errorhook:
	LOG_GETTID_LEAVE(ercd, TaskID);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.p_tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetTaskID);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetTaskID */

/*
 *  タスク状態の参照
 */
#ifdef TOPPERS_GetTaskState

StatusType
GetTaskState(TaskType TaskID, TaskStateRefType State)
{
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	CCB			*p_ccb;

	LOG_GETTST_ENTER(TaskID);
	CHECK_CALLEVEL(CALLEVEL_GETTASKSTATE);
	CHECK_ID(TaskID < tnum_task);
	CHECK_PARAM_POINTER(State);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	p_ccb = p_tcb->p_tinib->p_ccb;

	*State = (p_tcb == p_ccb->p_runtsk) ? RUNNING : p_tcb->tstat;

  exit_no_errorhook:
	LOG_GETTST_LEAVE(ercd, State);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.tskid = TaskID;
	get_my_p_ccb()->temp_errorhook_par2.p_stat = State;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetTaskState);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetTaskState */

/*
 *  満了処理専用タスクの起動
 *
 *  条件：OS割込み禁止状態で呼ばれる
 */
#ifdef TOPPERS_activate_task_action

StatusType
activate_task_action(TaskType TaskID)
{
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	CCB			*p_ccb;

	LOG_ACTTSK_ENTER(TaskID);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	p_ccb = p_tcb->p_tinib->p_ccb;

	acquire_tsk_lock(p_ccb);

	if (p_tcb->tstat == SUSPENDED) {
		if (make_active(p_tcb) != FALSE) {
			(void) dispatch_request(p_ccb);
		}
	}
	else {
		S_D_CHECK_LIMIT(p_tcb->actcnt < p_tcb->p_tinib->maxact);

		p_tcb->actcnt += 1U;
	}

  d_exit_no_errorhook:
	release_tsk_lock(p_ccb);
  exit_no_errorhook:
	LOG_ACTTSK_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	release_tsk_lock(p_ccb);
  exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ActivateTask);
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_activate_task_action */
