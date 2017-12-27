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
 *  $Id: event.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		イベント管理モジュール
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

#ifndef LOG_SETEVT_ENTER
#define LOG_SETEVT_ENTER(tskid, mask)
#endif /* LOG_SETEVT_ENTER */

#ifndef LOG_SETEVT_LEAVE
#define LOG_SETEVT_LEAVE(ercd)
#endif /* LOG_SETEVT_LEAVE */

#ifndef LOG_CLREVT_ENTER
#define LOG_CLREVT_ENTER(p_runtsk, mask)
#endif /* LOG_CLREVT_ENTER */

#ifndef LOG_CLREVT_LEAVE
#define LOG_CLREVT_LEAVE(ercd)
#endif /* LOG_CLREVT_LEAVE */

#ifndef LOG_GETEVT_ENTER
#define LOG_GETEVT_ENTER(tskid)
#endif /* LOG_GETEVT_ENTER */

#ifndef LOG_GETEVT_LEAVE
#define LOG_GETEVT_LEAVE(ercd, tskid, p_mask)
#endif /* LOG_GETEVT_LEAVE */

#ifndef LOG_WAIEVT_ENTER
#define LOG_WAIEVT_ENTER(p_runtsk, mask)
#endif /* LOG_WAIEVT_ENTER */

#ifndef LOG_WAIEVT_LEAVE
#define LOG_WAIEVT_LEAVE(ercd)
#endif /* LOG_WAIEVT_LEAVE */

/*
 *  イベントのセット
 */
#ifdef TOPPERS_SetEvent

StatusType
SetEvent(TaskType TaskID, EventMaskType Mask)
{
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	CCB			*p_ccb;
	boolean		dspreq = FALSE;

	LOG_SETEVT_ENTER(TaskID, Mask);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_SETEVENT);
	CHECK_ID(TaskID < tnum_task);
	CHECK_ACCESS(TaskID < tnum_exttask);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	p_ccb = p_tcb->p_tinib->p_ccb;

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);
	D_CHECK_STATE(p_tcb->tstat != SUSPENDED);

	p_tcb->curevt |= Mask;
	if ((p_tcb->curevt & p_tcb->waievt) != EVTMASK_NONE) {
		p_tcb->waievt = EVTMASK_NONE;
		if (make_runnable(p_tcb) != FALSE) {
			/*
			 * タスクtskidが他コアであればディスパッチ要求
			 * 自コアでかつタスクコンテキストから呼び出されていた場合dspreqを真に
			 */
			if ((dispatch_request(p_ccb) != FALSE) && (p_ccb->callevel_stat == TCL_TASK)) {
				dspreq = TRUE;
			}
		}
	}
	release_tsk_lock_and_dispatch(p_ccb, dspreq);
	x_nested_unlock_os_int();

  exit_no_errorhook:
	LOG_SETEVT_LEAVE(ercd);
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
	get_my_p_ccb()->temp_errorhook_par2.mask = Mask;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_SetEvent);
#else /* CFG_USE_ERRORHOOK */
  d_exit_no_errorhook:
	release_tsk_lock(p_ccb);
#endif /* CFG_USE_ERRORHOOK */
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
}

#endif /* TOPPERS_SetEvent */

/*
 *  イベントのクリア
 */
#ifdef TOPPERS_ClearEvent

StatusType
ClearEvent(EventMaskType Mask)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_CLREVT_ENTER(p_ccb->p_runtsk, Mask);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CLEAREVENT);
	CHECK_ACCESS(TSKID(p_ccb->p_runtsk) < tnum_exttask);

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);

	p_ccb->p_runtsk->curevt &= ~Mask;
	release_tsk_lock(p_ccb);

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_CLREVT_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.mask = Mask;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_ClearEvent);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_ClearEvent */

/*
 *  イベントの状態参照
 */
#ifdef TOPPERS_GetEvent

StatusType
GetEvent(TaskType TaskID, EventMaskRefType Event)
{
	StatusType	ercd = E_OK;
	TCB			*p_tcb;
	CCB			*p_ccb;

	LOG_GETEVT_ENTER(TaskID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_GETEVENT);
	CHECK_ID(TaskID < tnum_task);
	CHECK_ACCESS(TaskID < tnum_exttask);
	CHECK_PARAM_POINTER(Event);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	p_ccb = p_tcb->p_tinib->p_ccb;

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);

	/* 対象タスクが休止状態の場合はエラーとする */
	D_CHECK_STATE((p_tcb->tstat != SUSPENDED) || (p_tcb == (p_ccb->p_runtsk)));

	*Event = p_tcb->curevt;

  d_exit_no_errorhook:
	release_tsk_lock(p_ccb);
	x_nested_unlock_os_int();
  exit_no_errorhook:
	LOG_GETEVT_LEAVE(ercd, TaskID, Event);
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
	get_my_p_ccb()->temp_errorhook_par2.p_mask = Event;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetEvent);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetEvent */

/*
 *  イベント待ち
 */
#ifdef TOPPERS_WaitEvent

StatusType
WaitEvent(EventMaskType Mask)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb = get_my_p_ccb();

	LOG_WAIEVT_ENTER(p_ccb->p_runtsk, Mask);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_WAITEVENT);
	CHECK_ACCESS(TSKID(p_ccb->p_runtsk) < tnum_exttask);
	CHECK_RESOURCE(p_ccb->p_runtsk->p_lastrescb == NULL);
	CHECK_SPINLOCK(p_ccb->p_runtsk->p_lastspncb == NULL);

	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);
	if ((p_ccb->p_runtsk->curevt & Mask) == EVTMASK_NONE) {
		p_ccb->p_runtsk->curpri = p_ccb->p_runtsk->p_tinib->inipri;
		p_ccb->p_runtsk->tstat = WAITING;
		LOG_TSKSTAT(p_ccb->p_runtsk);
		p_ccb->p_runtsk->waievt = Mask;
		make_non_runnable(p_ccb);
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
	LOG_WAIEVT_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->temp_errorhook_par1.mask = Mask;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_WaitEvent);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_WaitEvent */

/*
 *  満了処理専用イベントのセット
 *
 *  条件：OS割込み禁止状態で呼ばれる
 */
#ifdef TOPPERS_set_event_action

StatusType
set_event_action(TaskType TaskID, EventMaskType Mask)
{
	StatusType	ercd;
	TCB			*p_tcb;
	CCB			*p_ccb;

	LOG_SETEVT_ENTER(TaskID, Mask);
	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);
	p_ccb = p_tcb->p_tinib->p_ccb;
	ercd = E_OK;

	acquire_tsk_lock(p_ccb);
	D_CHECK_STATE(p_tcb->tstat != SUSPENDED);

	p_tcb->curevt |= Mask;
	if ((p_tcb->curevt & p_tcb->waievt) != EVTMASK_NONE) {
		p_tcb->waievt = EVTMASK_NONE;
		if (make_runnable(p_tcb) != FALSE) {
			(void) dispatch_request(p_ccb);
		}
	}

  d_exit_no_errorhook:
	release_tsk_lock(p_ccb);
  exit_no_errorhook:
	LOG_SETEVT_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	release_tsk_lock(p_ccb);
  exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.tskid = TaskID;
	get_my_p_ccb()->temp_errorhook_par2.mask = Mask;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_SetEvent);
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_set_event_action */
