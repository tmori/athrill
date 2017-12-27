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
 *  $Id: osctl.c 630 2016-03-17 07:42:55Z ertl-ishikawa $
 */

/*
 *		OS制御モジュール
 */

#include "kernel_impl.h"
#include "interrupt.h"
#include "task.h"
#include "alarm.h"
#include "scheduletable.h"
#include "osap.h"

/*
 *  トレースログマクロのデフォルト定義
 */

#ifndef LOG_ERRHOOK_ENTER
#define LOG_ERRHOOK_ENTER(ercd)
#endif /* LOG_ERRHOOK_ENTER */

#ifndef LOG_ERRHOOK_LEAVE
#define LOG_ERRHOOK_LEAVE()
#endif /* LOG_ERRHOOK_LEAVE */

#ifndef LOG_PROHOOK_ENTER
#define LOG_PROHOOK_ENTER(ercd)
#endif /* LOG_PROHOOK_ENTER */

#ifndef LOG_PROHOOK_LEAVE
#define LOG_PROHOOK_LEAVE(pret)
#endif /* LOG_PROHOOK_LEAVE */

#ifndef LOG_SHUTHOOK_ENTER
#define LOG_SHUTHOOK_ENTER(ercd)
#endif /* LOG_SHUTHOOK_ENTER */

#ifndef LOG_SHUTHOOK_LEAVE
#define LOG_SHUTHOOK_LEAVE()
#endif /* LOG_SHUTHOOK_LEAVE */

/*
 *  各オブジェクトの初期化ブロックへのポインタ初期化
 *
 *  NULLの場合は割付けられたコアが起動していない
 */
#ifdef TOPPERS_p_inib_initialize

void
p_inib_initialize(void)
{
	TaskType			ti;
	AlarmType			ai;
	ScheduleTableType	si;
	CounterType			ci;
	ResourceType		ri;
	ISRType				ii;
	ApplicationType		oi;

	for (ti = 0U; ti < tnum_task; ti++) {
		get_tcb(ti)->p_tinib = NULL;
	}
	for (ai = 0U; ai < tnum_alarm; ai++) {
		get_almcb(ai)->p_alminib = NULL;
	}
	for (si = 0U; si < tnum_scheduletable; si++) {
		get_schtblcb(si)->p_schtblinib = NULL;
	}
	for (ci = 0U; ci < tnum_counter; ci++) {
		get_cntcb(ci)->p_cntinib = NULL;
	}
	for (ri = 0U; ri < tnum_stdresource; ri++) {
		get_rescb(ri)->p_resinib = NULL;
	}
	for (ii = 0U; ii < tnum_isr2; ii++) {
		get_isrcb(ii)->p_isrinib = NULL;
	}
	for (oi = 0U; oi < tnum_osap; oi++) {
		get_osapcb(oi)->p_osapinib = NULL;
	}
}

#endif /* TOPPERS_p_inib_initialize */

#ifdef CFG_USE_ERRORHOOK
/*
 *  エラーフックに渡す情報を格納する変数
 */
#ifdef TOPPERS_internal_call_errorhook

/*
 *  エラーフックの呼び出し
 */
void
internal_call_errorhook(StatusType ercd, OSServiceIdType svcid)
{
	CCB		*p_ccb = get_my_p_ccb();

	boolean	saved_run_trusted = p_ccb->run_trusted;

	if ((p_ccb->callevel_stat & (TCL_ERROR | TSYS_ISR1)) == TCL_NULL) {

#ifdef CFG_USE_GETSERVICEID
		p_ccb->_errorhook_svcid = svcid;
#endif /* CFG_USE_GETSERVICEID */

#ifdef CFG_USE_PARAMETERACCESS
		p_ccb->errorhook_par1 = p_ccb->_errorhook_par1;
		p_ccb->errorhook_par2 = p_ccb->_errorhook_par2;
		p_ccb->errorhook_par3 = p_ccb->_errorhook_par3;
#endif /* CFG_USE_PARAMETERACCESS */

		ENTER_CALLEVEL(TCL_ERROR);
		p_ccb->run_trusted = TRUE;

		LOG_ERRHOOK_ENTER(ercd);
		ErrorHook(ercd);
		LOG_ERRHOOK_LEAVE();
		LEAVE_CALLEVEL(TCL_ERROR);
	}

	p_ccb->run_trusted = saved_run_trusted;
}

#endif /* TOPPERS_internal_call_errorhook */

#endif /* CFG_USE_ERRORHOOK */

#ifdef CFG_USE_POSTTASKHOOK
/*
 *  ポストタスクフックの呼び出し
 */
#ifdef TOPPERS_call_posttaskhook

void
call_posttaskhook(void)
{
	boolean saved_run_trusted;
	CCB		*p_ccb = get_my_p_ccb();

	ENTER_CALLEVEL(TCL_PREPOST);
	saved_run_trusted = p_ccb->run_trusted;
	p_ccb->run_trusted = TRUE;

	PostTaskHook();

	p_ccb->run_trusted = saved_run_trusted;

	LEAVE_CALLEVEL(TCL_PREPOST);
}

#endif /* TOPPERS_call_posttaskhook */

#endif /* CFG_USE_POSTTASKHOOK */

#ifdef CFG_USE_PRETASKHOOK
/*
 *  プレタスクフックの呼び出し
 */
#ifdef TOPPERS_call_pretaskhook

void
call_pretaskhook(void)
{
	boolean saved_run_trusted;
	CCB		*p_ccb = get_my_p_ccb();

	ENTER_CALLEVEL(TCL_PREPOST);
	saved_run_trusted = p_ccb->run_trusted;
	p_ccb->run_trusted = TRUE;

	PreTaskHook();

	p_ccb->run_trusted = saved_run_trusted;

	LEAVE_CALLEVEL(TCL_PREPOST);
}

#endif /* TOPPERS_call_pretaskhook */

#endif /* CFG_USE_PRETASKHOOK */

#ifdef CFG_USE_STACKMONITORING

#ifdef TOPPERS_init_stack_magic_region

/*
 *  スタックモニタリング機能の初期化
 *  スタックモニタリング機能のためのマジックナンバー領域の初期化
 */
void
init_stack_magic_region(void)
{
	StackType	*p_stack_magic_region;
	CoreIdType	coreid = x_core_id();

	/*
	 *  スタックモニタリング機能のため，スタック成長方向考慮した
	 *  非タスクスタックのマジックナンバー領域の初期化
	 */
	p_stack_magic_region = TOPPERS_ISTK_MAGIC_REGION(_ostk_table[coreid], _ostksz_table[coreid]);
	*p_stack_magic_region = STACK_MAGIC_NUMBER;
}

#endif /* TOPPERS_init_stack_magic_region */

#endif /* CFG_USE_STACKMONITORING */

/*
 *  プロテクションフックの呼び出し
 *  引数の最上位ビットは，呼び出し箇所を示す役割がある
 */
#ifdef TOPPERS_call_protectionhk_main

void
call_protectionhk_main(StatusType protection_error)
{
	CCB						*p_ccb = get_my_p_ccb();

#ifdef CFG_USE_PROTECTIONHOOK

	boolean					saved_run_trusted;
	ProtectionReturnType	pret;

	/* プロテクションフック実行中に保護違反が発生した場合 */
	if ((p_ccb->callevel_stat & TCL_PROTECT) == TCL_PROTECT) {
		internal_shutdownallcores(E_OS_PROTECTION_FATAL);
	}

	/* 以下 プロテクションフックを呼出す処理 */
	ENTER_CALLEVEL(TCL_PROTECT);
	saved_run_trusted = p_ccb->run_trusted;

	p_ccb->run_trusted = TRUE;
	LOG_PROHOOK_ENTER(protection_error);
	pret = ProtectionHook(protection_error);
	LOG_PROHOOK_LEAVE(pret);
	p_ccb->run_trusted = saved_run_trusted;

	LEAVE_CALLEVEL(TCL_PROTECT);

	/* 以下 ProtectionHook 実行後の処理 */
	switch (pret) {
	case PRO_SHUTDOWN:
		internal_shutdownallcores(protection_error);
		break;
	case PRO_TERMINATETASKISR:
		if ((p_ccb->p_runtsk == NULL) || (p_ccb->pre_protection_supervised != FALSE)) {
			internal_shutdownallcores(E_OS_PROTECTION_FATAL);
		}
		else {
			/* タスクの場合 */
			force_terminate_task(p_ccb->p_runtsk);
		}
		break;
	case PRO_IGNORE:
		if (protection_error != E_OS_PROTECTION_EXCEPTION) {
			internal_shutdownallcores(E_OS_PROTECTION_FATAL);
		}
		break;
	case PRO_TERMINATEAPPL:
		if ((p_ccb->p_runosap == NULL) || (p_ccb->pre_protection_supervised != FALSE)) {
			internal_shutdownallcores(E_OS_PROTECTION_FATAL);
		}
		else {
			if ((p_ccb->p_runosap->osap_stat == APPLICATION_ACCESSIBLE) ||
				(p_ccb->p_runosap->osap_stat == APPLICATION_RESTARTING)) {
				force_term_osap(p_ccb->p_runosap, NO_RESTART, p_ccb, p_ccb);
			}
			else {
				/*
				 * アプリケーションが終了状態であれば，OSAP強制終了
				 * リスタートタスクが実行可能状態のはずであるので，
				 * 現在実行中のコンテキストを捨てて，ディスパッチする
				 * そして，コア間割込み→リスタートタスクを実行して　
				 * OSAPを強制終了する
				 */
				exit_and_dispatch_nohook();
			}
		}
		break;
	case PRO_TERMINATEAPPL_RESTART:
		if ((p_ccb->p_runosap == NULL) || (p_ccb->pre_protection_supervised != FALSE)) {
			internal_shutdownallcores(E_OS_PROTECTION_FATAL);
		}
		else {
			if ((p_ccb->p_runosap->osap_stat == APPLICATION_ACCESSIBLE) ||
				(p_ccb->p_runosap->osap_stat == APPLICATION_RESTARTING)) {
				if ((p_ccb->term_osap_id_map &
					 (0x1U << OSAPID(p_ccb->p_runosap))) == 0U) {
					force_term_osap(p_ccb->p_runosap, RESTART, p_ccb, p_ccb);
				}
				else {
					/*
					 * アプリケーションがRESTARTING'状態であれば，OSAP強制終了
					 * リスタートタスクが実行可能状態のはずであるので，
					 * 現在実行中のコンテキストを捨てて，ディスパッチする
					 * そして，コア間割込み→リスタートタスクを実行して　
					 * OSAPを強制終了する
					 */
					exit_and_dispatch_nohook();
				}
			}
			else {
				/*
				 * アプリケーションが終了状態であれば，OSAP強制終了
				 * リスタートタスクが実行可能状態のはずであるので，
				 * 現在実行中のコンテキストを捨てて，ディスパッチする
				 * そして，コア間割込み→リスタートタスクを実行して　
				 * OSAPを強制終了する
				 */
				exit_and_dispatch_nohook();
			}
		}
		break;
	default:
		internal_shutdownallcores(E_OS_PROTECTION_FATAL);
		break;
	}

#else /* CFG_USE_PROTECTIONHOOK */

	/*
	 *  プロテクションフックがコンフィギュレーション時に無効と
	 *  されている場合，OSは保護違反時処理としてOSシャットダウンを
	 *  行う
	 *  このとき，OSシャットダウンのパラメータとして，
	 *  違反の区別を示すエラーコードを指定する
	 */
	internal_shutdownallcores(protection_error);
#endif /* CFG_USE_PROTECTIONHOOK */

}

#endif /* TOPPERS_call_protectionhk_main */

#ifdef TOPPERS_internal_call_shtdwnhk

#ifdef CFG_USE_SHUTDOWNHOOK

void
internal_call_shtdwnhk(StatusType ercd)
{

	CCB	*p_ccb = get_my_p_ccb();

	/*
	 *  シャットダウンフック中のシャットダウンではシャットダウンフック
	 *  は呼び出さない
	 */
	if ((p_ccb->callevel_stat & TCL_SHUTDOWN) == TCL_NULL) {

		barrier_sync(4U, FALSE);

		p_ccb->p_runosap = NULL;

		/*
		 *  ShutdownHook の呼び出し
		 */
		ENTER_CALLEVEL(TCL_SHUTDOWN);
		p_ccb->run_trusted = TRUE;
		LOG_SHUTHOOK_ENTER(ercd);
		ShutdownHook(ercd);
		LOG_SHUTHOOK_LEAVE();
		LEAVE_CALLEVEL(TCL_SHUTDOWN);
	}
}
#endif /* CFG_USE_SHUTDOWNHOOK */

#endif /* TOPPERS_internal_call_shtdwnhk */

/*
 *  エラーフックOFF時，サービスID取得とパラメータ取得もOFFになる
 */
#ifdef TOPPERS_get_error_svcid
OSServiceIdType
get_error_svcid(void)
{
	return(get_my_p_ccb()->_errorhook_svcid);
}
#endif /* TOPPERS_get_error_svcid */

#ifdef TOPPERS_get_error_par
void
get_error_par(_ErrorHook_Par *p_errorhook_par, uint8 par_num)
{
	switch (par_num) {
	case 1U:
		*p_errorhook_par = get_my_p_ccb()->errorhook_par1;
		break;
	case 2U:
		*p_errorhook_par = get_my_p_ccb()->errorhook_par2;
		break;
	case 3U:
		*p_errorhook_par = get_my_p_ccb()->errorhook_par3;
		break;
	default:
		break;
	}
}
#endif /* TOPPERS_get_error_par */
