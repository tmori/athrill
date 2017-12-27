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
 *  $Id: osctl.c 432 2015-12-09 11:54:28Z nces-okajima $
 */

/*
 *		OS制御モジュール
 */

#include "kernel_impl.h"
#include "interrupt.h"
#include "task.h"

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

#ifndef LOG_STUTOS_ENTER
#define LOG_STUTOS_ENTER(ercd)
#endif /* LOG_STUTOS_ENTER */

#ifndef LOG_STUTOS_LEAVE
#define LOG_STUTOS_LEAVE()
#endif /* LOG_STUTOS_LEAVE */

#ifdef CFG_USE_ERRORHOOK
/*
 *  エラーフックに渡す情報を格納する変数
 */
#ifdef TOPPERS_internal_call_errorhook

#ifdef CFG_USE_GETSERVICEID
DEFINE_VAR_SEC_NOBITS(OSServiceIdType, _errorhook_svcid, ".srpw_bss_kernel");
#endif /* CFG_USE_GETSERVICEID */

#ifdef CFG_USE_PARAMETERACCESS
_ErrorHook_Par	_errorhook_par1;
_ErrorHook_Par	_errorhook_par2;
_ErrorHook_Par	_errorhook_par3;
DEFINE_VAR_SEC_NOBITS(_ErrorHook_Par, errorhook_par1, ".srpw_bss_kernel");
DEFINE_VAR_SEC_NOBITS(_ErrorHook_Par, errorhook_par2, ".srpw_bss_kernel");
DEFINE_VAR_SEC_NOBITS(_ErrorHook_Par, errorhook_par3, ".srpw_bss_kernel");
#endif /* CFG_USE_PARAMETERACCESS */

/*
 *  エラーフックの呼び出し
 */
void
internal_call_errorhook(StatusType ercd, OSServiceIdType svcid)
{

	boolean	saved_run_trusted = run_trusted;

	if ((callevel_stat & (TCL_ERROR | TSYS_ISR1)) == TCL_NULL) {

#ifdef CFG_USE_GETSERVICEID
		_errorhook_svcid = svcid;
#endif /* CFG_USE_GETSERVICEID */

#ifdef CFG_USE_PARAMETERACCESS
		errorhook_par1 = _errorhook_par1;
		errorhook_par2 = _errorhook_par2;
		errorhook_par3 = _errorhook_par3;
#endif /* CFG_USE_PARAMETERACCESS */

		ENTER_CALLEVEL(TCL_ERROR);
		run_trusted = TRUE;

		LOG_ERRHOOK_ENTER(ercd);
		ErrorHook(ercd);
		LOG_ERRHOOK_LEAVE();
		LEAVE_CALLEVEL(TCL_ERROR);

	}

	run_trusted = saved_run_trusted;
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

	ENTER_CALLEVEL(TCL_PREPOST);
	saved_run_trusted = run_trusted;
	run_trusted = TRUE;

	PostTaskHook();

	run_trusted = saved_run_trusted;

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

	ENTER_CALLEVEL(TCL_PREPOST);
	saved_run_trusted = run_trusted;
	run_trusted = TRUE;

	PreTaskHook();

	run_trusted = saved_run_trusted;

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
	StackType *p_stack_magic_region;

	/*
	 *  スタックモニタリング機能のため，スタック成長方向考慮した
	 *  非タスクスタックのマジックナンバー領域の初期化
	 */
	p_stack_magic_region = TOPPERS_ISTK_MAGIC_REGION(_ostk, _ostksz);
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
#ifdef CFG_USE_PROTECTIONHOOK

	boolean					saved_run_trusted;

	ProtectionReturnType	pret;

	/* プロテクションフック実行中に保護違反が発生した場合 */
	if ((callevel_stat & TCL_PROTECT) == TCL_PROTECT) {
		internal_shutdownos(E_OS_PROTECTION_FATAL);
	}

	/* 以下 プロテクションフックを呼出す処理 */

	ENTER_CALLEVEL(TCL_PROTECT);
	saved_run_trusted = run_trusted;

	run_trusted = TRUE;
	LOG_PROHOOK_ENTER(protection_error);
	pret = ProtectionHook(protection_error);
	LOG_PROHOOK_LEAVE(pret);
	run_trusted = saved_run_trusted;

	LEAVE_CALLEVEL(TCL_PROTECT);

	/* 以下 ProtectionHook 実行後の処理 */
	switch (pret) {
	case PRO_SHUTDOWN:
		internal_shutdownos(protection_error);
		break;
	case PRO_TERMINATETASKISR:
		if ((p_runtsk == NULL) || (pre_protection_supervised != FALSE)) {
			/* 信頼領域からのフック時はシャットダウン */
			internal_shutdownos(E_OS_PROTECTION_FATAL);
		}
		else {
			/* タスクの場合 */
			force_terminate_task(p_runtsk);
		}
		break;
	case PRO_IGNORE:
		if (protection_error != E_OS_PROTECTION_EXCEPTION) {
			internal_shutdownos(E_OS_PROTECTION_FATAL);
		}
		break;
	case PRO_TERMINATEAPPL:
		if ((p_runosap == NULL) || (pre_protection_supervised != FALSE)) {
			internal_shutdownos(E_OS_PROTECTION_FATAL);
		}
		else {
			force_term_osap(p_runosap, NO_RESTART);
		}
		break;
	case PRO_TERMINATEAPPL_RESTART:
		if ((p_runosap == NULL) || (pre_protection_supervised != FALSE)) {
			internal_shutdownos(E_OS_PROTECTION_FATAL);
		}
		else {
			force_term_osap(p_runosap, RESTART);
		}
		break;
	default:
		/* ProtectionHookから不正な値が返った場合 */
		internal_shutdownos(E_OS_PROTECTION_FATAL);
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
	internal_shutdownos(protection_error);
#endif /* CFG_USE_PROTECTIONHOOK */

}

#endif /* TOPPERS_call_protectionhk_main */

/*
 *  OS内部からのShutdownOSの呼び出し
 */
#ifdef TOPPERS_internal_shutdownos

void
internal_shutdownos(StatusType ercd)
{
	LOG_STUTOS_ENTER(ercd);

	x_nested_lock_os_int();

#ifdef CFG_USE_SHUTDOWNHOOK
	call_shutdownhook(ercd);
#endif /* CFG_USE_SHUTDOWNHOOK */

	/* 各モジュールの終了処理 */
	object_terminate();

	/* 全割込み禁止状態に移行 */
	x_lock_all_int();

	LOG_STUTOS_LEAVE();

	/* ターゲット依存の終了処理 */
	target_exit();

	/*
	 * ターゲット依存部から処理が返ってきた場合，
	 * 無限ループを行う
	 */
	while (1) {
	}

}

#endif /* TOPPERS_internal_shutdownos */

#ifdef TOPPERS_internal_call_shtdwnhk

#ifdef CFG_USE_SHUTDOWNHOOK

void
internal_call_shtdwnhk(StatusType ercd)
{

	/*
	 *  シャットダウンフック中のシャットダウンではシャットダウンフック
	 *  は呼び出さない
	 */
	if ((callevel_stat & TCL_SHUTDOWN) == TCL_NULL) {

		p_runosap = NULL;

		/*
		 *  ShutdownHook の呼び出し
		 */
		ENTER_CALLEVEL(TCL_SHUTDOWN);
		run_trusted = TRUE;
		LOG_SHUTHOOK_ENTER(ercd);
		ShutdownHook(ercd);
		LOG_SHUTHOOK_LEAVE();
		LEAVE_CALLEVEL(TCL_SHUTDOWN);

	}
}
#endif /* CFG_USE_SHUTDOWNHOOK */

#endif /* TOPPERS_internal_call_shtdwnhk */
