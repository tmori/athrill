/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2015 by Witz Corporation
 *  Copyright (C) 2014-2015 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2015 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2015 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015 by SUZUKI MOTOR CORPORATION
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
 *  $Id: task_manage.c 464 2015-12-10 11:16:08Z witz-itoyo $
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
	OSAPCB		*p_osapcb;

	LOG_ACTTSK_ENTER(TaskID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_ACTIVATETASK);
	CHECK_ID(TaskID < tnum_task);
	p_tcb = get_tcb(TaskID);
	CHECK_RIGHT(p_tcb->p_tinib->acsbtmp);

	p_osapcb = p_tcb->p_tinib->p_osapcb;
	x_nested_lock_os_int();

	/* 起動するタスク所属のOSAPの状態をチェック */
	D_CHECK_ACCESS((p_osapcb->osap_stat == APPLICATION_ACCESSIBLE) ||
				   (p_osapcb == p_runosap));

	if (p_tcb->tstat == SUSPENDED) {
		if ((make_active(p_tcb) != FALSE) && (callevel_stat == TCL_TASK)) {
			dispatch();
		}
	}
	else {
		S_D_CHECK_LIMIT(p_tcb->actcnt < p_tcb->p_tinib->maxact);

		p_tcb->actcnt += 1U;
	}

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_ACTTSK_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ActivateTask);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ActivateTask */

/*
 *  自タスクの終了
 */
#ifdef TOPPERS_TerminateTask

StatusType
TerminateTask(void)
{
	StatusType ercd = E_OK;

	LOG_TERTSK_ENTER();
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_TERMINATETASK);
	CHECK_RESOURCE(p_runtsk->p_lastrescb == NULL);

	x_nested_lock_os_int();
	/*
	 *  内部リソースの解放は優先度を下げるだけなので，ここでは
	 *  何もしなくてよい
	 */

	suspend();

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
	OSAPCB		*p_osapcb;

	LOG_CHNTSK_ENTER(TaskID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHAINTASK);
	CHECK_RESOURCE(p_runtsk->p_lastrescb == NULL);
	CHECK_ID(TaskID < tnum_task);
	p_tcb = get_tcb(TaskID);
	CHECK_RIGHT(p_tcb->p_tinib->acsbtmp);

	p_osapcb = p_tcb->p_tinib->p_osapcb;
	x_nested_lock_os_int();

	/* 起動するタスク所属のOSAPの状態をチェック */
	D_CHECK_ACCESS((p_osapcb->osap_stat == APPLICATION_ACCESSIBLE) ||
				   (p_osapcb == p_runosap));

	if (p_tcb == p_runtsk) {
		make_non_runnable();
		(void) make_active(p_runtsk);
	}
	else {
		/*
		 *  エラー時に副作用が残らないように，エラーチェックは
		 *  タスク終了処理の前に行う必要がある
		 */
		S_D_CHECK_LIMIT((p_tcb->tstat == SUSPENDED)
						|| (p_tcb->actcnt < p_tcb->p_tinib->maxact));

		suspend();

		if (p_tcb->tstat == SUSPENDED) {
			(void) make_active(p_tcb);
		}
		else {
			p_tcb->actcnt += 1U;
		}
	}

	LOG_CHNTSK_LEAVE(E_OK);
	exit_and_dispatch();
	ASSERT_NO_REACHED;

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ChainTask);
#endif /* CFG_USE_ERRORHOOK */

  d_exit_no_errorhook:
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
	StatusType ercd = E_OK;

	LOG_SCHED_ENTER();
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_SCHEDULE);
	CHECK_RESOURCE(p_runtsk->p_lastrescb == NULL);

	x_nested_lock_os_int();
	if (p_runtsk->p_tinib->inipri > nextpri) {
		p_runtsk->curpri = p_runtsk->p_tinib->inipri;
		preempt();
		dispatch();
		p_runtsk->curpri = p_runtsk->p_tinib->exepri;
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
	StatusType ercd = E_OK;

	LOG_GETTID_ENTER();
	CHECK_CALLEVEL(CALLEVEL_GETTASKID);
	CHECK_PARAM_POINTER(TaskID);
	CHECK_MEM_WRITE(TaskID, TaskType);
	*TaskID = (p_runtsk == NULL) ? INVALID_TASK : TSKID(p_runtsk);

  exit_no_errorhook:
	LOG_GETTID_LEAVE(ercd, TaskID);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.p_tskid = TaskID;
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
	OSAPCB		*p_osapcb;

	LOG_GETTST_ENTER(TaskID);
	CHECK_CALLEVEL(CALLEVEL_GETTASKSTATE);
	CHECK_ID(TaskID < tnum_task);
	CHECK_PARAM_POINTER(State);
	CHECK_MEM_WRITE(State, TaskStateType);
	p_tcb = get_tcb(TaskID);
	CHECK_RIGHT(p_tcb->p_tinib->acsbtmp);

	p_osapcb = p_tcb->p_tinib->p_osapcb;
	x_nested_lock_os_int();

	/* 起動するタスク所属のOSAPの状態をチェック */
	D_CHECK_ACCESS((p_osapcb->osap_stat == APPLICATION_ACCESSIBLE) ||
				   (p_osapcb == p_runosap));
	*State = (p_tcb == p_runtsk) ? RUNNING : p_tcb->tstat;

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_GETTST_LEAVE(ercd, State);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.tskid = TaskID;
	_errorhook_par2.p_stat = State;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetTaskState);
	goto d_exit_no_errorhook;
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
activate_task_action(OSAPCB *p_expire_osapcb, TaskType TaskID)
{
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	OSAPCB		*p_osapcb;

	LOG_ACTTSK_ENTER(TaskID);
	p_tcb = get_tcb(TaskID);

	p_osapcb = p_tcb->p_tinib->p_osapcb;
	/* 満了点所属のOSAP及び起動するタスク所属のOSAPの状態をチェック */
	D_CHECK_ACCESS((p_osapcb->osap_stat == APPLICATION_ACCESSIBLE) || (p_expire_osapcb == p_osapcb));

	if (p_tcb->tstat == SUSPENDED) {
		(void) make_active(p_tcb);
	}
	else {
		S_D_CHECK_LIMIT(p_tcb->actcnt < p_tcb->p_tinib->maxact);

		p_tcb->actcnt += 1U;
	}

  d_exit_no_errorhook:
	LOG_ACTTSK_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ActivateTask);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */

}

#endif /* TOPPERS_activate_task_action */
