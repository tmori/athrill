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
 *  $Id: osctl.c 727 2017-01-23 09:27:59Z witz-itoyo $
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
OSServiceIdType	errorhook_svcid;
#endif /* CFG_USE_GETSERVICEID */

#ifdef CFG_USE_PARAMETERACCESS
ErrorHook_Par	temp_errorhook_par1;
ErrorHook_Par	temp_errorhook_par2;
ErrorHook_Par	temp_errorhook_par3;
ErrorHook_Par	errorhook_par1;
ErrorHook_Par	errorhook_par2;
ErrorHook_Par	errorhook_par3;
#endif /* CFG_USE_PARAMETERACCESS */

/*
 *  エラーフックの呼び出し
 */
void
internal_call_errorhook(StatusType ercd, OSServiceIdType svcid)
{

	if ((callevel_stat & (TCL_ERROR | TSYS_ISR1)) == TCL_NULL) {

#ifdef CFG_USE_GETSERVICEID
		errorhook_svcid = svcid;
#endif /* CFG_USE_GETSERVICEID */

#ifdef CFG_USE_PARAMETERACCESS
		errorhook_par1 = temp_errorhook_par1;
		errorhook_par2 = temp_errorhook_par2;
		errorhook_par3 = temp_errorhook_par3;
#endif /* CFG_USE_PARAMETERACCESS */

		ENTER_CALLEVEL(TCL_ERROR);
		LOG_ERRHOOK_ENTER(ercd);
		ErrorHook(ercd);
		LOG_ERRHOOK_LEAVE();
		LEAVE_CALLEVEL(TCL_ERROR);
	}
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
	ENTER_CALLEVEL(TCL_PREPOST);
	PostTaskHook();
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
	ENTER_CALLEVEL(TCL_PREPOST);
	PreTaskHook();
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
	TaskType	i;
	StackType	*p_stack_magic_region;

	/*
	 *  スタックモニタリング機能のため，スタック成長方向考慮した
	 *  非タスクスタックのマジックナンバー領域の初期化
	 */
	p_stack_magic_region = TOPPERS_ISTK_MAGIC_REGION(ostk, ostksz);
	*p_stack_magic_region = STACK_MAGIC_NUMBER;

	/*
	 *  スタックモニタリング機能のため，スタック成長方向考慮した
	 *  各タスクスタックのマジックナンバー領域の初期化
	 *  スタック共有したタスクのマジックナンバー領域が合わせっている
	 */
	for (i = 0U; i < tnum_task; i++) {
		p_stack_magic_region =
			TOPPERS_TSTK_MAGIC_REGION(&tinib_table[i]);
		*p_stack_magic_region = STACK_MAGIC_NUMBER;
	}
}

#endif /* TOPPERS_init_stack_magic_region */

#endif /* CFG_USE_STACKMONITORING */

/*
 *  プロテクションフックの呼び出し
 */
#ifdef TOPPERS_call_protectionhk_main

void
call_protectionhk_main(StatusType ercd)
{

#ifdef CFG_USE_PROTECTIONHOOK

	ProtectionReturnType pret;

	/* プロテクションフック実行中に保護違反が発生した場合 */
	if ((callevel_stat & TCL_PROTECT) == TCL_PROTECT) {
		internal_shutdownos(E_OS_PROTECTION_FATAL);
	}

	ENTER_CALLEVEL(TCL_PROTECT);
	LOG_PROHOOK_ENTER(ercd);
	pret = ProtectionHook(ercd);
	LOG_PROHOOK_LEAVE(pret);

	LEAVE_CALLEVEL(TCL_PROTECT);

	/* 以下 ProtectionHook 実行後の処理 */
	switch (pret) {
	case PRO_SHUTDOWN:
		internal_shutdownos(ercd);
		break;
	case PRO_IGNORE:
		if (ercd != E_OS_PROTECTION_EXCEPTION) {
			internal_shutdownos(E_OS_PROTECTION_FATAL);
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
	internal_shutdownos(ercd);
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

		ENTER_CALLEVEL(TCL_SHUTDOWN);
		LOG_SHUTHOOK_ENTER(ercd);
		ShutdownHook(ercd);
		LOG_SHUTHOOK_LEAVE();
		LEAVE_CALLEVEL(TCL_SHUTDOWN);

	}
}
#endif /* CFG_USE_SHUTDOWNHOOK */

#endif /* TOPPERS_internal_call_shtdwnhk */
