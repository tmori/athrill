/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2004-2015 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2015 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2015 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2015 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2015 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2015 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2015 by Witz Corporation
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
 *  $Id: osap.c 485 2015-12-17 08:21:50Z witz-itoyo $
 */

#include "kernel_impl.h"
#include "check.h"
#include "osap.h"
#include "task.h"
#include "interrupt.h"
#include "counter.h"
#include "alarm.h"
#include "resource.h"
#include "scheduletable.h"
#include "spinlock.h"
#include "mc.h"

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

#ifndef LOG_GETAST_ENTER
#define LOG_GETAST_ENTER(Application)
#endif /* LOG_GETAST_ENTER */

#ifndef LOG_GETAST_LEAVE
#define LOG_GETAST_LEAVE(ercd, p_value)
#endif /* LOG_GETAST_LEAVE */

#ifndef LOG_CALTFN_ENTER
#define LOG_CALTFN_ENTER(FunctionIndex)
#endif /* LOG_CALTFN_ENTER */

#ifndef LOG_CALTFN_LEAVE
#define LOG_CALTFN_LEAVE(FunctionIndex)
#endif /* LOG_CALTFN_LEAVE */

#ifndef LOG_TFN_ENTER
#define LOG_TFN_ENTER(FunctionIndex)
#endif /* LOG_TFN_ENTER */

#ifndef LOG_TFN_LEAVE
#define LOG_TFN_LEAVE(FunctionIndex, ercd)
#endif /* LOG_TFN_LEAVE */

#ifndef LOG_CHKTSKACS_ENTER
#define LOG_CHKTSKACS_ENTER(ApplID, TaskID)
#endif /* LOG_CHKTSKACS_ENTER */

#ifndef LOG_CHKTSKACS_LEAVE
#define LOG_CHKTSKACS_LEAVE(access)
#endif /* LOG_CHKTSKACS_LEAVE */

#ifndef LOG_CHKISRACS_ENTER
#define LOG_CHKISRACS_ENTER(ApplID, ISRID)
#endif /* LOG_CHKISRACS_ENTER */

#ifndef LOG_CHKISRACS_LEAVE
#define LOG_CHKISRACS_LEAVE(access)
#endif /* LOG_CHKISRACS_LEAVE */

#ifndef LOG_CHKALMACS_ENTER
#define LOG_CHKALMACS_ENTER(ApplID, AlarmID)
#endif /* LOG_CHKALMACS_ENTER */

#ifndef LOG_CHKALMACS_LEAVE
#define LOG_CHKALMACS_LEAVE(access)
#endif /* LOG_CHKALMACS_LEAVE */

#ifndef LOG_CHKRESACS_ENTER
#define LOG_CHKRESACS_ENTER(ApplID, ResID)
#endif /* LOG_CHKRESACS_ENTER */

#ifndef LOG_CHKRESACS_LEAVE
#define LOG_CHKRESACS_LEAVE(access)
#endif /* LOG_CHKRESACS_LEAVE */

#ifndef LOG_CHKCNTACS_ENTER
#define LOG_CHKCNTACS_ENTER(ApplID, CounterID)
#endif /* LOG_CHKCNTACS_ENTER */

#ifndef LOG_CHKCNTACS_LEAVE
#define LOG_CHKCNTACS_LEAVE(access)
#endif /* LOG_CHKCNTACS_LEAVE */

#ifndef LOG_CHKSCHTBLACS_ENTER
#define LOG_CHKSCHTBLACS_ENTER(ApplID, ScheduleTableID)
#endif /* LOG_CHKSCHTBLACS_ENTER */

#ifndef LOG_CHKSCHTBLACS_LEAVE
#define LOG_CHKSCHTBLACS_LEAVE(access)
#endif /* LOG_CHKSCHTBLACS_LEAVE */

#ifndef LOG_CHKSPNACS_ENTER
#define LOG_CHKSPNACS_ENTER(ApplID, SpinlockId)
#endif /* LOG_CHKSPNACS_ENTER */

#ifndef LOG_CHKSPNACS_LEAVE
#define LOG_CHKSPNACS_LEAVE(access)
#endif /* LOG_CHKSPNACS_LEAVE */

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

#ifndef LOG_ALLOWACCESS_ENTER
#define LOG_ALLOWACCESS_ENTER()
#endif /* LOG_ALLOWACCESS_ENTER */

#ifndef LOG_ALLOWACCESS_LEAVE
#define LOG_ALLOWACCESS_LEAVE(ercd)
#endif /* LOG_ALLOWACCESS_LEAVE */

#ifndef LOG_TERMINATEAPPLICATION_ENTER
#define LOG_TERMINATEAPPLICATION_ENTER(Application, RestartOption)
#endif /* LOG_TERMINATEAPPLICATION_ENTER */

#ifndef LOG_TERMINATEAPPLICATION_LEAVE
#define LOG_TERMINATEAPPLICATION_LEAVE(ercd)
#endif /* LOG_TERMINATEAPPLICATION_LEAVE */

#ifdef TOPPERS_force_term_osap
/* 内部関数のプロトタイプ宣言 */
static void force_term_osap_main(void);
#endif /* TOPPERS_force_term_osap */

#ifdef TOPPERS_osap_initialize

/*
 *  OSアプリケーション管理モジュールの初期化
 */
void
osap_initialize(void)
{
	ApplicationType	i;
	CoreIdType		coreid;
	const OSAPINIB	*p_osapinib;
	OSAPCB			*p_osapcb;

	coreid = x_core_id();
	for (i = 0U; i < tnum_osap; i++) {
		p_osapinib = &osapinib_table[i];
		if (p_osapinib->coreid == coreid) {
			p_osapcb = p_osapcb_table[i];
			p_osapcb->p_osapinib = p_osapinib;
			p_osapcb->osap_stat = APPLICATION_ACCESSIBLE;
		}
	}
}

#endif /* TOPPERS_osap_initialize */

/*
 *  実行状態のOSアプリケーション ID取得
 */
#ifdef TOPPERS_GetApplicationID

ApplicationType
GetApplicationID(void)
{
	ApplicationType id;
	StatusType		ercd;
	CCB				*p_ccb = get_my_p_ccb();

	LOG_GETOSAPID_ENTER();

	CHECK_CALLEVEL(CALLEVEL_GETAPPLICATIONID);

	id = (p_ccb->p_runosap == NULL) ? INVALID_OSAPPLICATION : OSAPID(p_ccb->p_runosap);

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
 *  アプリケーションを利用可能な状態にするシステムサービス
 */
#ifdef TOPPERS_GetApplicationState

StatusType
GetApplicationState(ApplicationType Application, ApplicationStateRefType Value)
{
	StatusType	ercd = E_OK;
	OSAPCB		*p_osapcb;

	LOG_GETAST_ENTER(Application);

	CHECK_CALLEVEL(CALLEVEL_GETAPPLICATIONSTATE);
	CHECK_PARAM_POINTER(Value);
	CHECK_MEM_WRITE(Value, ApplicationStateType);
	CHECK_ID(Application < tnum_osap);
	p_osapcb = get_osapcb(Application);
	CHECK_CORE(p_osapcb->p_osapinib != NULL);

	*Value = p_osapcb->osap_stat;

  exit_no_errorhook:
	LOG_GETAST_LEAVE(ercd, Value);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = Application;
	get_my_p_ccb()->_errorhook_par2.p_appstat = Value;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetApplicationState);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetApplicationState */

#ifdef TOPPERS_CallTrustedFunction

StatusType
CallTrustedFunction(TrustedFunctionIndexType FunctionIndex,
					TrustedFunctionParameterRefType FunctionParams)
{
	StatusType		ercd = E_OK;
	const TFINIB	*p_tfinib;
	boolean			saved_run_trusted;
#ifdef CFG_USE_PROTECTIONHOOK
	boolean			saved_calltfn;
#endif /* CFG_USE_PROTECTIONHOOK */
	CCB				*p_ccb = get_my_p_ccb();

	LOG_CALTFN_ENTER(FunctionIndex);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CALLTRUSTEDFUNCTION);

	/* ファンクションIDのチェック */
	CHECK_SERVICEID(FunctionIndex < tnum_tfn);

	p_tfinib = &(tfinib_table[FunctionIndex]);
	/* 同一コア呼び出しのチェック */
	CHECK_ACCESS(p_tfinib->coreid == x_core_id());

	/* 使用するスタックのチェック */
	ercd = trustedfunc_stack_check(p_tfinib->tf_stksz);
	CHECK_NO_ERCD(ercd == E_OK);

	saved_run_trusted = p_ccb->run_trusted;
	p_ccb->run_trusted = TRUE;

#ifdef CFG_USE_PROTECTIONHOOK
	/* 信頼関数呼び出し中フラグ設定 */
	if (p_ccb->callevel_stat == TCL_TASK) {
		saved_calltfn = p_ccb->p_runtsk->calltfn;
		p_ccb->p_runtsk->calltfn = TRUE;
	}
	else {
		saved_calltfn = p_ccb->p_runisr->calltfn;
		p_ccb->p_runisr->calltfn = TRUE;
	}
#endif /* CFG_USE_PROTECTIONHOOK */

	LOG_TFN_ENTER(FunctionIndex);
	/* 信頼関数実行 */
	ercd = p_tfinib->trs_func(FunctionIndex, FunctionParams);
	LOG_TFN_LEAVE(FunctionIndex, ercd);

#ifdef CFG_USE_STACKMONITORING
	if ((p_ccb->callevel_stat & TCL_ISR2) != TCL_NULL) {
		if ((*TOPPERS_ISTK_MAGIC_REGION(_ostk_table[p_ccb->coreid], _ostksz_table[p_ccb->coreid])) != STACK_MAGIC_NUMBER) {
			x_nested_lock_os_int();
			p_ccb->pre_protection_supervised = TRUE;
			call_protectionhk_main(E_OS_STACKFAULT);
			ASSERT_NO_REACHED;
		}
	}
	else {
		if ((*TOPPERS_SSTK_MAGIC_REGION(p_ccb->p_runtsk->p_tinib)) != STACK_MAGIC_NUMBER) {
			x_nested_lock_os_int();
			p_ccb->pre_protection_supervised = TRUE;
			call_protectionhk_main_stkchg(E_OS_STACKFAULT);
			ASSERT_NO_REACHED;
		}
	}
#endif /* CFG_USE_STACKMONITORING */

#ifdef CFG_USE_PROTECTIONHOOK
	/* 信頼関数呼び出し中フラグ戻す */
	if (p_ccb->callevel_stat == TCL_TASK) {
		p_ccb->p_runtsk->calltfn = saved_calltfn;
	}
	else {
		p_ccb->p_runisr->calltfn = saved_calltfn;
	}
#endif /* CFG_USE_PROTECTIONHOOK */

	p_ccb->run_trusted = saved_run_trusted;

	/* 信頼関数がエラーになった場合もエラーフックを呼び出す */
	CHECK_NO_ERCD(ercd == E_OK);

  exit_no_errorhook:
	LOG_CALTFN_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	p_ccb->_errorhook_par1.tfnid = FunctionIndex;
	p_ccb->_errorhook_par2.tfnpr = FunctionParams;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CallTrustedFunction);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_CallTrustedFunction */

/*
 *  アクセスチェック関数
 */

/*
 *  OSアプリケーションのオブジェクトのアクセス権限チェックシステムサービス
 */

/*
 *  ビットマップチェックマクロ
 *   このファイル内のみで使用するため，ここで定義している
 */

#define	CHECK_OSAP_ACS(p_osapinib, btmp)	(ObjectAccessType) ((((p_osapinib)->osap_trusted) != FALSE) || \
																(((p_osapinib)->btptn & (btmp)) != 0U))

/*
 *  タスク用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckTaskAccess

ObjectAccessType
CheckTaskAccess(ApplicationType ApplID, TaskType TaskID)
{
	ObjectAccessType	access;
	StatusType			ercd;
	TCB					*p_tcb;

	LOG_CHKTSKACS_ENTER(ApplID, TaskID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID(TaskID < tnum_task);

	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_tcb->p_tinib->acsbtmp);

  exit_finish:
	LOG_CHKTSKACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.tskid = TaskID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckTaskAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckTaskAccess */

/*
 *  ISR用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckISRAccess

ObjectAccessType
CheckISRAccess(ApplicationType ApplID, ISRType ISRID)
{
	ObjectAccessType	access;
	StatusType			ercd;
	ISRCB				*p_isrcb;

	LOG_CHKISRACS_ENTER(ApplID, ISRID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID(ISRID < tnum_isr2);

	p_isrcb = get_isrcb(ISRID);
	CHECK_CORE(p_isrcb->p_isrinib != NULL);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_isrcb->p_isrinib->acsbtmp);

  exit_finish:
	LOG_CHKISRACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.isrid = ISRID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckISRAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckISRAccess */

/*
 *  アラーム用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckAlarmAccess

ObjectAccessType
CheckAlarmAccess(ApplicationType ApplID, AlarmType AlarmID)
{
	ObjectAccessType	access;
	StatusType			ercd;
	ALMCB				*p_almcb;

	LOG_CHKALMACS_ENTER(ApplID, AlarmID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID(AlarmID < tnum_alarm);

	p_almcb = get_almcb(AlarmID);
	CHECK_CORE(p_almcb->p_alminib != NULL);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_almcb->p_alminib->acsbtmp);

  exit_finish:
	LOG_CHKALMACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.almid = AlarmID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckAlarmAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckAlarmAccess */

/*
 *  リソース用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckResourceAccess

ObjectAccessType
CheckResourceAccess(ApplicationType ApplID, ResourceType ResID)
{
	ObjectAccessType	access;
	StatusType			ercd;
	RESCB				*p_rescb;

	LOG_CHKRESACS_ENTER(ApplID, ResID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID(ResID < tnum_stdresource);

	p_rescb = get_rescb(ResID);
	CHECK_CORE(p_rescb->p_resinib != NULL);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_rescb->p_resinib->acsbtmp);

  exit_finish:
	LOG_CHKRESACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.resid = ResID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckResourceAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckResourceAccess */

/*
 *  カウンタ用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckCounterAccess

ObjectAccessType
CheckCounterAccess(ApplicationType ApplID, CounterType CounterID)
{
	ObjectAccessType	access;
	StatusType			ercd;
	CNTCB				*p_cntcb;

	LOG_CHKCNTACS_ENTER(ApplID, CounterID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID(CounterID < tnum_counter);

	p_cntcb = get_cntcb(CounterID);
	CHECK_CORE(p_cntcb->p_cntinib != NULL);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_cntcb->p_cntinib->acsbtmp);

  exit_finish:
	LOG_CHKCNTACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.cntid = CounterID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckCounterAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckCounterAccess */

/*
 *  スケジュールテーブル用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckScheduleTableAccess

ObjectAccessType
CheckScheduleTableAccess(ApplicationType ApplID, ScheduleTableType ScheduleTableID)
{
	ObjectAccessType	access;
	StatusType			ercd;
	SCHTBLCB			*p_schtblcb;

	LOG_CHKSCHTBLACS_ENTER(ApplID, ScheduleTableID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);

	p_schtblcb = get_schtblcb(ScheduleTableID);
	CHECK_CORE(p_schtblcb->p_schtblinib != NULL);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_schtblcb->p_schtblinib->acsbtmp);

  exit_finish:
	LOG_CHKSCHTBLACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.schtblid = ScheduleTableID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckScheduleTableAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckScheduleTableAccess */

/*
 *  スピンロック用アクセス権限チェックシステムサービス
 */
#ifdef TOPPERS_CheckSpinlockAccess

ObjectAccessType
CheckSpinlockAccess(ApplicationType ApplID, SpinlockIdType SpinlockId)
{
	ObjectAccessType	access;
	StatusType			ercd;
	SPNCB				*p_spncb;

	LOG_CHKSPNACS_ENTER(ApplID, SpinlockId);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTACCESS);
	CHECK_ID(ApplID < tnum_osap);
	CHECK_ID((SpinlockId != INVALID_SPINLOCK) && (SpinlockId <= tnum_spinlock));

	p_spncb = get_spncb(SpinlockId);

	access = CHECK_OSAP_ACS(get_osapinib(ApplID), p_spncb->p_spninib->acsbtmp);

  exit_finish:
	LOG_CHKSPNACS_LEAVE(access);
	return(access);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.applid = ApplID;
	get_my_p_ccb()->_errorhook_par2.spnid = SpinlockId;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckSpinlockAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	access = NO_ACCESS;
	goto exit_finish;
}

#endif /* TOPPERS_CheckSpinlockAccess */


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
	StatusType		ercd;
	TCB				*p_tcb;

	LOG_CHKTSKOWN_ENTER(TaskID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(TaskID < tnum_task);

	p_tcb = get_tcb(TaskID);
	CHECK_CORE(p_tcb->p_tinib != NULL);

	owner = OSAPID(p_tcb->p_tinib->p_osapcb);

  exit_finish:
	LOG_CHKTSKOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.tskid = TaskID;
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
	StatusType		ercd;
	ISRCB			*p_isrcb;

	LOG_CHKISROWN_ENTER(ISRID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(ISRID < tnum_isr2);

	p_isrcb = get_isrcb(ISRID);
	CHECK_CORE(p_isrcb->p_isrinib != NULL);

	owner = OSAPID(p_isrcb->p_isrinib->p_osapcb);

  exit_finish:
	LOG_CHKISROWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.isrid = ISRID;
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
	StatusType		ercd;
	ALMCB			*p_almcb;

	LOG_CHKALMOWN_ENTER(AlarmID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(AlarmID < tnum_alarm);

	p_almcb = get_almcb(AlarmID);
	CHECK_CORE(p_almcb->p_alminib != NULL);

	owner = OSAPID(p_almcb->p_alminib->p_osapcb);

  exit_finish:
	LOG_CHKALMOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.almid = AlarmID;
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
	StatusType		ercd;
	CNTCB			*p_cntcb;

	LOG_CHKCNTOWN_ENTER(CounterID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(CounterID < tnum_counter);

	p_cntcb = get_cntcb(CounterID);
	CHECK_CORE(p_cntcb->p_cntinib != NULL);

	owner = OSAPID(p_cntcb->p_cntinib->p_osapcb);

  exit_finish:
	LOG_CHKCNTOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.cntid = CounterID;
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
	StatusType		ercd;
	SCHTBLCB		*p_schtblcb;

	LOG_CHKSCHTBLOWN_ENTER(ScheduleTableID);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_CHECKOBJECTOWNERSHIP);
	CHECK_ID(ScheduleTableID < tnum_scheduletable);

	p_schtblcb = get_schtblcb(ScheduleTableID);
	CHECK_CORE(p_schtblcb->p_schtblinib != NULL);

	owner = OSAPID(p_schtblcb->p_schtblinib->p_osapcb);

  exit_finish:
	LOG_CHKSCHTBLOWN_LEAVE(owner);
	return(owner);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	get_my_p_ccb()->_errorhook_par1.schtblid = ScheduleTableID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_CheckScheduleTableOwnership);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	owner = INVALID_OSAPPLICATION;
	goto exit_finish;
}

#endif /* TOPPERS_CheckScheduleTableOwnership */


/*
 *  自OSAPを利用可能な状態にする
 */
#ifdef TOPPERS_AllowAccess

StatusType
AllowAccess(void)
{
	StatusType	ercd = E_OK;
	CCB			*p_ccb;

	LOG_ALLOWACCESS_ENTER();
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_ALLOWACCESS);
	p_ccb = get_my_p_ccb();
	/* ここに来た場合p_runosap(自OSAP)がNULLではない */
	CHECK_STATE(p_ccb->p_runosap->osap_stat == APPLICATION_RESTARTING);

	x_nested_lock_os_int();

	p_ccb->p_runosap->osap_stat = APPLICATION_ACCESSIBLE;

	x_nested_unlock_os_int();

  exit_no_errorhook:
	LOG_ALLOWACCESS_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	call_errorhook(ercd, OSServiceId_AllowAccess);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */
	goto exit_no_errorhook;
}

#endif /* TOPPERS_AllowAccess */


/*
 *  指定OSAPを終了/再起動する
 */
#ifdef TOPPERS_TerminateApplication

StatusType
TerminateApplication(ApplicationType Application, RestartType RestartOption)
{
	StatusType	ercd = E_OK;
	CCB			*my_p_ccb = get_my_p_ccb();
	CCB			*p_ccb;
	OSAPCB		*p_osapcb;

	LOG_TERMINATEAPPLICATION_ENTER(Application, RestartOption);

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_TERMINATEAPPLICATION);
	CHECK_ID((Application < tnum_osap) && (get_osapinib(Application)->osap_trusted == FALSE));
	p_osapcb = get_osapcb(Application);
	CHECK_CORE(p_osapcb->p_osapinib != NULL);
	CHECK_VALUE((RestartOption == NO_RESTART) || ((RestartOption == RESTART) &&
												  (p_osapcb->p_osapinib->p_restart_tcb->p_tinib->task != NULL)));

	CHECK_ACCESS((my_p_ccb->run_trusted != FALSE) || (p_osapcb == my_p_ccb->p_runosap));

	p_ccb = get_p_ccb(p_osapcb->p_osapinib->coreid);
	x_nested_lock_os_int();
	acquire_tsk_lock(p_ccb);

	S_D_CHECK_STATE((p_osapcb->osap_stat == APPLICATION_ACCESSIBLE) ||
					((p_osapcb->osap_stat == APPLICATION_RESTARTING) &&
					 (p_osapcb == my_p_ccb->p_runosap) &&
					 (RestartOption == NO_RESTART)));

	force_term_osap(p_osapcb, RestartOption, p_ccb, my_p_ccb);

	x_nested_unlock_os_int();

  exit_no_errorhook:
	LOG_TERMINATEAPPLICATION_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
	release_tsk_lock(p_ccb);
	goto errorhook_start;

  exit_errorhook:
	x_nested_lock_os_int();
  errorhook_start:
#ifdef CFG_USE_PARAMETERACCESS
	my_p_ccb->_errorhook_par1.applid = Application;
	my_p_ccb->_errorhook_par2.restartoption = RestartOption;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_TerminateApplication);
#else /* CFG_USE_ERRORHOOK */
  d_exit_no_errorhook:
	release_tsk_lock(p_ccb);
#endif /* CFG_USE_ERRORHOOK */
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
}

#endif /* TOPPERS_TerminateApplication */

/*
 *  指定OSAPを終了/再起動する内部関数
 */
#ifdef TOPPERS_force_term_osap

static void
force_term_osap_main(void)
{
	CCB		*p_my_ccb = get_my_p_ccb();
	OSAPCB	*p_osapcb;

	p_osapcb = p_my_ccb->p_runosap;

	acquire_cnt_lock(p_my_ccb);
	force_term_osap_counter(p_osapcb);
	release_cnt_lock(p_my_ccb);
	force_term_osap_task(p_osapcb);

	acquire_tsk_lock(p_my_ccb);
	p_osapcb->p_osapinib->p_restart_tcb->actcnt = 0U;
	if (p_osapcb->osap_stat == APPLICATION_RESTARTING) {
		(void) make_active(p_osapcb->p_osapinib->p_restart_tcb);
	}
	else {
		/* リスタートタスクをSUSPEND状態とする */
		p_osapcb->p_osapinib->p_restart_tcb->tstat = SUSPENDED;
	}

	/* 仮のリスタートタスクは役目を終えるため、キューの処理を実施 */
	search_schedtsk(p_my_ccb);
	release_tsk_lock(p_my_ccb);

	/* タスクコンテキストの設定 */
	p_my_ccb->callevel_stat = TCL_TASK;

	if (p_my_ccb->sus_os_cnt > 0U) {
		p_my_ccb->sus_os_cnt = 0U;
		x_nested_unlock_os_int();
	}
	/* 自OSAPから呼び出しのため，呼び出し元に戻らない */
	exit_and_dispatch_nohook();
}

/*
 *  リスタートタスクのpcにforce_term_osap_mainのアドレスを格納する
 */
void
force_term_osap(OSAPCB *p_osapcb, RestartType RestartOption, CCB *p_ccb, CCB *p_my_ccb)
{
	TCB				*p_tcb;
	PriorityType	remove_task_pri;

	/* osap_statの更新 */
	if (RestartOption == RESTART) {
		p_osapcb->osap_stat = APPLICATION_RESTARTING;
		if (p_osapcb->p_osapinib->p_restart_tcb->p_tinib->task == NULL) {
			/* リスタートタスクの設定がないにもかかわらず，*/
			/* RESTARTオプションを設定した場合はシャットダウンを実施する */
			internal_shutdownallcores(E_OS_PROTECTION_FATAL);
		}
	}
	else {
		p_osapcb->osap_stat = APPLICATION_TERMINATED;
	}

	p_tcb = p_osapcb->p_osapinib->p_restart_tcb;

	/* リスタートタスクが既にREADY状態の場合はレディキューから削除する */
	if (p_tcb->tstat == READY) {
		/* リスタートタスクの終了 */
		p_tcb->actcnt = 0U;
#ifdef CFG_USE_PROTECTIONHOOK
		p_tcb->calltfn = FALSE;
#endif /* CFG_USE_PROTECTIONHOOK */
		/* curpriとinipriが異なる値でレディキューに繋がれている場合への対応 */
		remove_task_pri = p_tcb->curpri;

		/* スピンロック取得したままの場合はスピンロックを解放する */
		if (p_tcb->p_lastspncb != NULL) {
			force_release_spinlocks(p_tcb->p_lastspncb);
		}

		/* リソース確保したままの場合，リソース解放 */
		release_taskresources(p_tcb);

		/* カウンタの状態を初期化する */
		cancel_taskcounters(p_tcb);

		/* 対象タスクをレディキューから削除する */
		if (p_tcb != p_ccb->p_schedtsk) {
			remove_task_from_queue(p_ccb, p_tcb, remove_task_pri);
			/* p_schedtskをキューの先頭に退避 */
			move_schedtsk(p_ccb);
		}
		/*
		 * OSAP強制終了用のリスタートタスクが，他コアからの
		 * ActivateTaskなどによって状態が変化することを避ける
		 * ため，SUSPENDED状態にはしない
		 */
	}
	else if (p_ccb->p_schedtsk != NULL) {
		/* p_schedtskがNULLでなければ，キューの先頭に退避 */
		move_schedtsk(p_ccb);
	}
	else {
		/* p_schedtskがNULLの場合は何もしない */
	}

	/* リスタートタスクのpcにforce_term_osap_mainのアドレスを格納 */
	activate_force_term_osap_main(p_tcb);

	/* curpriを最高優先度に変更 */
	p_tcb->curpri = TPRI_MAXTASK;

	/* p_schedtskにp_restart_tcbを格納 */
	p_ccb->p_schedtsk = p_tcb;

	if (p_ccb == p_my_ccb) {
		/* 終了/再起動するOSAPの要求ビットを解除 */
		p_ccb->term_osap_id_map &= ~((uint32) 0x00000001U << OSAPID(p_osapcb));
		/*
		 * 自コアのアラームとスケジュールテーブルを
		 * 停止する
		 * アラーム，スケジュールテーブルは，自身のコアと
		 * 同じコアのカウンタにしか接続できないため，
		 * 自身のコアのカウンタロックを取得するだけでよい
		 */
		acquire_cnt_lock(p_ccb);
		force_term_osap_alarm(p_osapcb);
		force_term_osap_schtbl(p_osapcb);
		release_cnt_lock(p_ccb);
	}
	else {
		/*
		 * 他コアのアラームとスケジュールテーブルは，
		 * 他コアで停止する
		 * RESTARTING'状態のOSAPがアラームを起動してしまう
		 * ことがあるため
		 */
		/* 終了/再起動するOSAPの要求ビットを設定 */
		p_ccb->term_osap_id_map |= (uint32) 0x00000001U << OSAPID(p_osapcb);
		target_ici_raise(p_ccb->coreid);
	}

	release_tsk_lock(p_ccb);

	if ((p_ccb != p_my_ccb) ||
		((p_ccb->callevel_stat & TCL_ISR2) != TCL_NULL)) {
		/* 他コアのOSAPを強制終了する場合はディスパッチしない */
		/* C2ISRから呼び出された場合は,C2ISRの出口処理で */
		/* ディスパッチを行うため，ここでは実施しない    */
	}
	else if (p_osapcb != p_ccb->p_runosap) {
		/* 他のOSAPを終了/再起動する場合は,ディスパッチを行う */
		dispatch();
	}
	else {
		/* 自OSAPを終了/再起動する場合は,呼び出し元に戻らない */
		exit_and_dispatch_nohook();
	}
}

#endif /* TOPPERS_force_term_osap */

/*
 *  指定OSAPを終了/再起動する内部関数(コア間割込みからの呼び出し)
 */
#ifdef TOPPERS_term_osap_for_other_core

void
term_osap_for_other_core(void)
{
	CCB				*my_p_ccb = get_my_p_ccb();
	uint32			req_id_map;
	uint16			reqmap_half;
	ApplicationType	osapid;
	OSAPCB			*p_osapcb;

	/*
	 *  OSAPの終了/再起動用コア間割込み要求チェック
	 */
	acquire_tsk_lock(my_p_ccb);
	req_id_map = my_p_ccb->term_osap_id_map;

	while (req_id_map != 0U) {
		release_tsk_lock(my_p_ccb);
		/* 終了/再起動するOSAPの要因を検索(最下位ビット優先) */
		reqmap_half = (uint16) (req_id_map & 0x0000ffffU);
		if (reqmap_half != 0U) {
			osapid = (ApplicationType) bitmap_search(reqmap_half);
		}
		else {
			reqmap_half = (uint16) ((req_id_map & 0xffff0000U) >> 16U);
			osapid = (ApplicationType) (bitmap_search(reqmap_half) + 16U);
		}
		/* 要因ビットとOSAPのID関係は上記式で計算できる */
		p_osapcb = get_osapcb(osapid);

		acquire_cnt_lock(my_p_ccb);
		force_term_osap_alarm(p_osapcb);
		force_term_osap_schtbl(p_osapcb);
		release_cnt_lock(my_p_ccb);

		/* 探索された要因の要求ビットを落とす */
		acquire_tsk_lock(my_p_ccb);
		my_p_ccb->term_osap_id_map &= ~((uint32) 0x00000001U << osapid);
		req_id_map = my_p_ccb->term_osap_id_map;
	}
	release_tsk_lock(my_p_ccb);
}
#endif /* TOPPERS_term_osap_for_other_core */
