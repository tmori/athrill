/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2017 by Witz Corporation
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
 *  $Id: osap.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

#include "kernel_impl.h"
#include "check.h"
#include "osap.h"
#include "task.h"
#include "interrupt.h"
#include "counter.h"
#include "alarm.h"
#include "scheduletable.h"

/*
 *		OSアプリケーション管理モジュール
 */

/*
 *  トレースマクロのデフォルト定義
 */
#ifndef LOG_GETOSAPID_ENTER
#define LOG_GETOSAPID_ENTER()
#endif /* LOG_GETOSAPID_ENTER */

#ifndef LOG_GETOSAPID_LEAVE
#define LOG_GETOSAPID_LEAVE(id)
#endif /* LOG_GETOSAPID_LEAVE */

#ifndef LOG_CHKTSKOWN_ENTER
#define LOG_CHKTSKOWN_ENTER(TaskID)
#endif /* LOG_CHKTSKOWN_ENTER */

#ifndef LOG_CHKTSKOWN_LEAVE
#define LOG_CHKTSKOWN_LEAVE(owner)
#endif /* LOG_CHKTSKOWN_LEAVE */

#ifndef LOG_CHKISROWN_ENTER
#define LOG_CHKISROWN_ENTER(ISRID)
#endif /* LOG_CHKISROWN_ENTER */

#ifndef LOG_CHKISROWN_LEAVE
#define LOG_CHKISROWN_LEAVE(owner)
#endif /* LOG_CHKISROWN_LEAVE */

#ifndef LOG_CHKALMOWN_ENTER
#define LOG_CHKALMOWN_ENTER(AlarmID)
#endif /* LOG_CHKALMOWN_ENTER */

#ifndef LOG_CHKALMOWN_LEAVE
#define LOG_CHKALMOWN_LEAVE(owner)
#endif /* LOG_CHKALMOWN_LEAVE */

#ifndef LOG_CHKCNTOWN_ENTER
#define LOG_CHKCNTOWN_ENTER(CounterID)
#endif /* LOG_CHKCNTOWN_ENTER */

#ifndef LOG_CHKCNTOWN_LEAVE
#define LOG_CHKCNTOWN_LEAVE(owner)
#endif /* LOG_CHKCNTOWN_LEAVE */

#ifndef LOG_CHKSCHTBLOWN_ENTER
#define LOG_CHKSCHTBLOWN_ENTER(ScheduleTableID)
#endif /* LOG_CHKSCHTBLOWN_ENTER */

#ifndef LOG_CHKSCHTBLOWN_LEAVE
#define LOG_CHKSCHTBLOWN_LEAVE(owner)
#endif /* LOG_CHKSCHTBLOWN_LEAVE */

/*
 *  実行状態のOSアプリケーション ID取得
 */
#ifdef TOPPERS_GetApplicationID

ApplicationType
GetApplicationID(void)
{
	ApplicationType id;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType		ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	CCB				*p_ccb = get_my_p_ccb();

	LOG_GETOSAPID_ENTER();

	CHECK_CALLEVEL(CALLEVEL_GETAPPLICATIONID);

	if ((p_ccb->callevel_stat & TCL_ISR2) != TCL_NULL) {
		id = OSAPID(p_ccb->p_runisr->p_isrinib->p_osapinib);
	}
	else if ((p_ccb->callevel_stat & TCL_TASK) != TCL_NULL) {
		id = OSAPID(p_ccb->p_runtsk->p_tinib->p_osapinib);
	}
	else {
		id = INVALID_OSAPPLICATION;
	}

  exit_finish:
	LOG_GETOSAPID_LEAVE(id);
	return(id);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_GetApplicationID);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	id = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_GetApplicationID */

/*
 *  OSアプリケーションのオブジェクトの所有チェックシステムサービス
 */

/*
 *  タスク用オブジェクト所有チェックシステムサービス
 */
#ifdef TOPPERS_CheckTaskOwnership

ApplicationType
CheckTaskOwnership(TaskType TaskID)
{
	ApplicationType	owner;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType		ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	TCB				*p_tcb;

	LOG_CHKTSKOWN_ENTER(TaskID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(TaskID < tnum_task);

	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);

	owner = OSAPID(p_tcb->p_tinib->p_osapinib);

  exit_finish:
	LOG_CHKTSKOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckTaskOwnership);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	owner = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_CheckTaskOwnership */

/*
 *  ISR用オブジェクト所有チェックシステムサービス
 */
#ifdef TOPPERS_CheckISROwnership

ApplicationType
CheckISROwnership(ISRType ISRID)
{
	ApplicationType	owner;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType		ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	ISRCB			*p_isrcb;

	LOG_CHKISROWN_ENTER(ISRID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(ISRID < tnum_isr2);

	p_isrcb = get_isrcb(ISRID);
	CHECK_CORE(p_isrcb->p_isrinib != NULL);

	owner = OSAPID(p_isrcb->p_isrinib->p_osapinib);

  exit_finish:
	LOG_CHKISROWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.isrid = ISRID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckISROwnership);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	owner = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_CheckISROwnership */

/*
 *  アラーム用オブジェクト所有チェックシステムサービス
 */
#ifdef TOPPERS_CheckAlarmOwnership

ApplicationType
CheckAlarmOwnership(AlarmType AlarmID)
{
	ApplicationType	owner;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType		ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	ALMCB			*p_almcb;

	LOG_CHKALMOWN_ENTER(AlarmID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(AlarmID < tnum_alarm);

	p_almcb = get_almcb(AlarmID);
	CHECK_CORE(p_almcb->p_alminib != NULL);

	owner = OSAPID(p_almcb->p_alminib->p_osapinib);

  exit_finish:
	LOG_CHKALMOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.almid = AlarmID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckAlarmOwnership);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	owner = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_CheckAlarmOwnership */

/*
 *  カウンタ用オブジェクト所有チェックシステムサービス
 */
#ifdef TOPPERS_CheckCounterOwnership

ApplicationType
CheckCounterOwnership(CounterType CounterID)
{
	ApplicationType	owner;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType		ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	CNTCB			*p_cntcb;

	LOG_CHKCNTOWN_ENTER(CounterID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(CounterID < tnum_counter);

	p_cntcb = get_cntcb(CounterID);
	CHECK_CORE(p_cntcb->p_cntinib != NULL);

	owner = OSAPID(p_cntcb->p_cntinib->p_osapinib);

  exit_finish:
	LOG_CHKCNTOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.cntid = CounterID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckCounterOwnership);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	owner = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_CheckCounterOwnership */

/*
 *  スケジュールテーブル用オブジェクト所有チェックシステムサービス
 */
#ifdef TOPPERS_CheckScheduleTableOwnership

ApplicationType
CheckScheduleTableOwnership(ScheduleTableType ScheduleTableID)
{
	ApplicationType	owner;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType		ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */
	SCHTBLCB		*p_schtblcb;

	LOG_CHKSCHTBLOWN_ENTER(ScheduleTableID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);

	p_schtblcb = get_schtblcb(ScheduleTableID);
	CHECK_CORE(p_schtblcb->p_schtblinib != NULL);

	owner = OSAPID(p_schtblcb->p_schtblinib->p_osapinib);

  exit_finish:
	LOG_CHKSCHTBLOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->temp_errorhook_par1.schtblid = ScheduleTableID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckScheduleTableOwnership);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	owner = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_CheckScheduleTableOwnership */
