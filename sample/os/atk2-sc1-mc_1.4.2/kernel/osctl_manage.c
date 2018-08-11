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
 *  $Id: osctl_manage.c 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *		OS管理モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "interrupt.h"
#include "mc.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_GETAAM_ENTER
#define LOG_GETAAM_ENTER()
#endif /* LOG_GETAAM_ENTER */

#ifndef LOG_GETAAM_LEAVE
#define LOG_GETAAM_LEAVE(mode)
#endif /* LOG_GETAAM_LEAVE */

#ifndef LOG_STAOS_ENTER
#define LOG_STAOS_ENTER(mode)
#endif /* LOG_STAOS_ENTER */

#ifndef LOG_STAOS_LEAVE
#define LOG_STAOS_LEAVE()
#endif /* LOG_STAOS_LEAVE */

#ifndef LOG_STAHOOK_ENTER
#define LOG_STAHOOK_ENTER()
#endif /* LOG_STAHOOK_ENTER */

#ifndef LOG_STAHOOK_LEAVE
#define LOG_STAHOOK_LEAVE()
#endif /* LOG_STAHOOK_LEAVE */

#ifdef TOPPERS_StartOS

/*
 *  OS実行制御のための変数
 */
AppModeType	appmodeid;                                              /* アプリケーションモードID */

/*
 *  ファイル名，行番号の参照用の変数
 */
const char8	*fatal_file_name = NULL;                    /* ファイル名 */
sint32		fatal_line_num = 0;                         /* 行番号 */


/*
 *  OSの起動
 */

void
StartOS(AppModeType Mode)
{
	static AppModeType	appmodeid_table[TotalNumberOfCores];             /* 各コアのアプリケーションモードID */
	static boolean		os_controlled[TotalNumberOfCores] = {
		FALSE
	};                                                                   /* コアがOS管理かどうか（FALSEが0でなければ正しく初期化されない） */

	CCB					*p_ccb = get_my_p_ccb();
	CoreIdType			coreid;
	CoreIdType			my_coreid = x_core_id();
	AppModeType			appmodeid_local = DONOTCARE;

	LOG_STAOS_ENTER(Mode);
	/*
	 *  コアIDが，TotalNumberOfCoresより大きい場合は，
	 *  CCBが存在しないので，何もせずにreturn
	 */
#ifdef CFG_USE_EXTENDEDSTATUS
	if (my_coreid < TotalNumberOfCores) {
		if (p_ccb->kerflg != FALSE) {
			/* OS起動中はエラーフックを呼ぶ */
#ifdef CFG_USE_ERRORHOOK
			x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
			p_ccb->temp_errorhook_par1.mode = Mode;
#endif /* CFG_USE_PARAMETERACCESS */
			call_errorhook(E_OS_ACCESS, OSServiceId_StartOS);
			x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */
		}
		else {
#endif /* CFG_USE_EXTENDEDSTATUS */

			/* 全割込み禁止状態に移行 */
			x_lock_all_int();

#ifdef CFG_USE_STACKMONITORING
			/*
			 *  スタックモニタリング機能の初期化
			 *  スタックモニタリング機能のためのマジックナンバー領域の初期化
			 */
			init_stack_magic_region();
#endif /* CFG_USE_STACKMONITORING */

			/*
			 * マスタコアについてもTRUEとするため，
			 * StartCoreではなくここで処理する
			 */
			os_controlled[my_coreid] = TRUE;

			/* アプリケーションモードの設定 */
			appmodeid_table[my_coreid] = Mode;

			if (x_sense_mcore() != FALSE) {
				p_inib_initialize();
			}

			barrier_sync(1U, FALSE);

			/* アプリケーションモードのチェック */
			for (coreid = 0U; coreid < TotalNumberOfCores; coreid++) {
				if ((os_controlled[coreid] != FALSE) && (appmodeid_table[coreid] != DONOTCARE)) {
					/* 異なるアプリケーションモードが指定されたら無限ループ */
					if ((appmodeid_local != appmodeid_table[coreid]) && (appmodeid_local != DONOTCARE)) {
						while (1) {
						}
					}
					appmodeid_local = appmodeid_table[coreid];
				}
			}
			/* すべてDONOTCAREなら無限ループ */
			if (appmodeid_local == DONOTCARE) {
				while (1) {
				}
			}
			appmodeid = appmodeid_local;

			/*
			 *  コア管理ブロック初期化
			 */
			ccb_initialize();

			/* ターゲット依存の初期化 */
			target_initialize();

			/* 各モジュールの初期化 */
			object_initialize();

			p_ccb->callevel_stat = TCL_NULL;

			/* カーネル動作中 */
			p_ccb->kerflg = TRUE;

			/*
			 *  Modeが不正であった場合，OSシャットダウンを行う
			 *  この時，スタートアップフックは呼び出されない
			 */
			if (appmodeid >= tnum_appmode) {
				/*
				 *  internal_shutdownallcoresを呼ぶ前にOS割込み禁止状態へ
				 *  全割込み禁止状態解除
				 */
				x_nested_lock_os_int();
				x_unlock_all_int();
				internal_shutdownallcores(E_OS_MODE);
			}

			barrier_sync(2U, FALSE);

#ifdef CFG_USE_STARTUPHOOK
			/* OS割込み禁止状態にし，全割込み禁止状態解除 */
			x_nested_lock_os_int();
			x_unlock_all_int();

			/*
			 *  StartupHook の呼び出し
			 */
			ENTER_CALLEVEL(TCL_STARTUP);
			LOG_STAHOOK_ENTER();
			StartupHook();
			LOG_STAHOOK_LEAVE();
			LEAVE_CALLEVEL(TCL_STARTUP);

			/* 元の割込みマスク優先度と全割込み禁止状態に */
			x_lock_all_int();
			x_nested_unlock_os_int();
#endif /* CFG_USE_STARTUPHOOK */

			ENTER_CALLEVEL(TCL_TASK);

			/*
			 * スタートアップフック中にOSのシャットダウンを行うと，以下の
			 * バリア同期で待っているコアに割込みが入らないため，シャット
			 * ダウン処理を続けることができなくなる．これを防ぐために，
			 * シャットダウン要求が出ているかをチェックする．
			 */
			barrier_sync(3U, TRUE);
			LOG_STAOS_LEAVE();
			start_dispatch();
			ASSERT_NO_REACHED;
#ifdef CFG_USE_EXTENDEDSTATUS
		}
	}
#endif /* CFG_USE_EXTENDEDSTATUS */
}

#endif /* TOPPERS_StartOS */

/*
 *  現在のアプリケーションモードの取得
 */
#ifdef TOPPERS_GetActiveApplicationMode

AppModeType
GetActiveApplicationMode(void)
{
	AppModeType	appmode;
#if defined(CFG_USE_EXTENDEDSTATUS) || defined(CFG_USE_ERRORHOOK)
	StatusType ercd;
#endif /* CFG_USE_EXTENDEDSTATUS || CFG_USE_ERRORHOOK */

	LOG_GETAAM_ENTER();
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_GETACTIVEAPPMODE);

	appmode = appmodeid;

  exit_finish:
	LOG_GETAAM_LEAVE(appmode);
	return(appmode);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
	/*
	 *  エラー発生時はINVALID_APPMODETYPEが返るが，エラーが発生したのか実行中の
	 *  C2ISRが存在しないのか区別するため，エラーフックを呼ぶ
	 */
	call_errorhook(ercd, OSServiceId_GetActiveApplicationMode);
	x_nested_unlock_os_int();
#endif /* CFG_USE_ERRORHOOK */

  exit_no_errorhook:
	appmode = INVALID_APPMODETYPE;
	goto exit_finish;
}

#endif /* TOPPERS_GetActiveApplicationMode */

/*
 *  保護違反を起こした処理単位の取得
 */
#ifdef TOPPERS_GetFaultyContext

FaultyContextType
GetFaultyContext(void)
{
	FaultyContextType faultycontext = FC_INVALID;

#ifdef CFG_USE_PROTECTIONHOOK
	CCB    *p_ccb = get_my_p_ccb();
	uint16 callevel_stat = p_ccb->callevel_stat;

	if ((callevel_stat & CALLEVEL_GETFAULTYCONTEXT) != 0U) {

		/* C1ISR以外で発生 */
		if ((callevel_stat & TSYS_ISR1) == 0U) {
			/* フック中に発生 */
			if ((callevel_stat & (TCL_ERROR | TCL_PREPOST | TCL_STARTUP | TCL_SHUTDOWN)) != 0U) {
				/* システム定義フック中に発生 */
				faultycontext = FC_SYSTEM_HOOK;
			}
			else if ((callevel_stat & TCL_ISR2) != 0U) {
				if ((callevel_stat & TCL_ALRMCBAK) == 0U) {
					faultycontext = FC_C2ISR;
				}
			}
			else if ((callevel_stat & TCL_TASK) != 0U) {
				if ((callevel_stat & TCL_ALRMCBAK) == 0U) {
					faultycontext = FC_TASK;
				}
			}
			else {
				/* 上記以外の場合，処理は行わない(戻り値：FC_INVALID) */
			}
		}
	}
#endif /* CFG_USE_PROTECTIONHOOK */

	return(faultycontext);
}
#endif /* TOPPERS_GetFaultyContext */
