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
 *  $Id: trace_config.h 439 2015-12-09 12:33:41Z aisincom-nishioka $
 */

/*
 *		トレースログに関する設定
 */

#ifndef TOPPERS_TRACE_CONFIG_H
#define TOPPERS_TRACE_CONFIG_H

#ifdef TOPPERS_ENABLE_TRACE

/*
 *  トレースログバッファのサイズ
 */
#ifndef TCNT_TRACE_BUFFER
#define TCNT_TRACE_BUFFER	UINT_C(8192)

#endif /* TCNT_TRACE_BUFFER */

/*
 *  トレース時刻の取得方法
 */
#ifndef GET_UTIM
#define GET_UTIM()		(get_tim_utime())
#endif /* GET_UTIM */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  トレースログのデータ構造
 *
 *  システムログ機能のログ情報のデータ構造と同じものを用いる
 */
#include "t_syslog.h"
typedef	SYSLOG TRACE;

typedef	uint32 TraceModeType;                 /* サービスコールの動作モード */

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  トレースモードの定義
 */
#define TRACE_STOP			UINT_C(0x00)    /* トレース停止 */
#define TRACE_RINGBUF		UINT_C(0x01)    /* リングバッファモード */
#define TRACE_AUTOSTOP		UINT_C(0x02)    /* 自動停止モード */
#define TRACE_CLEAR			UINT_C(0x04)    /* トレースログのクリア */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  トレースログ機能の初期化
 *
 *  トレースログ機能を初期化する．初期化ルーチンとして登録することを想
 *  定している．引数により次の動作を行う
 *
 *  TRACE_STOP：初期化のみでトレースは開始しない
 *  TRACE_RINGBUF：リングバッファモードでトレースを開始
 *  TRACE_AUTOSTOP：自動停止モードでトレースを開始
 */
extern void trace_initialize(uintptr exinf);

/*
 *  トレースログの開始
 *
 *  トレースログの記録を開始／停止する．引数により次の動作を行う
 *
 *  TRACE_STOP：トレースを停止
 *  TRACE_RINGBUF：リングバッファモードでトレースを開始
 *  TRACE_AUTOSTOP：自動停止モードでトレースを開始
 *  TRACE_CLEAR：トレースログをクリア
 */
extern StatusType trace_sta_log(TraceModeType mode);

/*
 *  トレースログの読出し
 */
extern StatusType trace_rea_log(TRACE *p_trace);

/*
 *  トレースログのダンプ（trace_dump.c）
 *
 *  トレースログをダンプする．終了処理ルーチンとして登録することも想定
 *  している．引数として，ダンプ先となる文字出力関数へのポインタを渡す
 *  ターゲット依存の低レベル文字出力を利用する場合には，target_putcを渡
 *  す
 */
extern void trace_dump(void (*exinf)(char8 c));

/*
 *  トレースログを出力するためのライブラリ関数
 */
extern void trace_write_0(uint32 type);
extern void trace_write_1(uint32 type, const uintptr arg1);
extern void trace_write_2(uint32 type, uintptr arg1, uintptr arg2);
extern void trace_write_3(uint32 type, uintptr arg1, uintptr arg2, uintptr arg3);
extern void trace_write_4(uint32 type, uintptr arg1, uintptr arg2, uintptr arg3, uintptr arg4);
extern void trace_write_5(uint32 type, uintptr arg1, uintptr arg2, uintptr arg3, uintptr arg4, uintptr arg5);

/*
 *  トレースログを出力するためのマクロ
 */

#define trace_0(type)								trace_write_0((type))
#define trace_1(type, arg1)							trace_write_1((type), (arg1))
#define trace_2(type, arg1, arg2)					trace_write_2((type), (arg1), (arg2))
#define trace_3(type, arg1, arg2, arg3)				trace_write_3((type), (arg1), (arg2), (arg3))
#define trace_4(type, arg1, arg2, arg3, arg4)		trace_write_4((type), (arg1), (arg2), (arg3), (arg4))
#define trace_5(type, arg1, arg2, arg3, arg4, arg5)	trace_write_5((type), (arg1), (arg2), (arg3), (arg4), (arg5))

#endif /* TOPPERS_MACRO_ONLY */

/*
 *  トレースログ方法の設定
 */

/*
 *  割込みサービスルーチンの前後
 */
#define LOG_ISR_ENTER(isrid)	trace_1(LOG_TYPE_ISR | LOG_ENTER, (isrid))
#define LOG_ISR_LEAVE(isrid)	trace_1(LOG_TYPE_ISR | LOG_LEAVE, (isrid))

/*
 *  アラームハンドラの前後
 */
#define LOG_ALM_ENTER(p_almcb)	trace_1(LOG_TYPE_ALM | LOG_ENTER, (uintptr) (p_almcb))
#define LOG_ALM_LEAVE(p_almcb)	trace_1(LOG_TYPE_ALM | LOG_LEAVE, (uintptr) (p_almcb))

/*
 *  スケジュールテーブル満了処理の前後
 */
#define LOG_SCHTBL_ENTER(p_schtblcb)	trace_1(LOG_TYPE_SCHTBL | LOG_ENTER, (uintptr) (p_schtblcb))
#define LOG_SCHTBL_LEAVE(p_schtblcb)	trace_1(LOG_TYPE_SCHTBL | LOG_LEAVE, (uintptr) (p_schtblcb))

/*
 *  タスクの状態変更
 */
#define LOG_TSKSTAT(p_tcb)	trace_2(LOG_TYPE_TSKSTAT, (uintptr) (p_tcb), (uintptr) (p_tcb)->tstat)

/*
 *  ディスパッチャの前後
 */
#define LOG_DSP_ENTER(p_tcb)	trace_1(LOG_TYPE_DSP | LOG_ENTER, (p_tcb))
#define LOG_DSP_LEAVE(p_tcb)	trace_1(LOG_TYPE_DSP | LOG_LEAVE, (p_tcb))

/*
 * ユーザーマーク
 */
#define LOG_TYPE_USER_MARK	UINT_C(0x100)
#define LOG_USER_MARK(str)	trace_1(LOG_TYPE_USER_MARK, &(str))


/*
 *  システムコール
 */

/*
 *  タスク管理機能
 */

#define LOG_ACTTSK_ENTER(tskid)			trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_ACTIVATETASK, (uintptr) (tskid))
#define LOG_ACTTSK_LEAVE(ercd)			trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_ACTIVATETASK, (uintptr) (ercd))
#define LOG_TERTSK_ENTER()				trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_TERMINATETASK)
#define LOG_TERTSK_LEAVE(ercd)			trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_TERMINATETASK, (uintptr) (ercd))
#define LOG_CHNTSK_ENTER(tskid)			trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CHAINTASK, (uintptr) (tskid))
#define LOG_CHNTSK_LEAVE(ercd)			trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHAINTASK, (uintptr) (ercd))
#define LOG_SCHED_ENTER()				trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_SCHEDULE)
#define LOG_SCHED_LEAVE(ercd)			trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_SCHEDULE, (uintptr) (ercd))
#define LOG_GETTID_ENTER()				trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_GETTASKID)
#define LOG_GETTID_LEAVE(ercd, p_tskid)	trace_3(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETTASKID, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_tskid)) : (0U)))
#define LOG_GETTST_ENTER(tskid)			trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETTASKSTATE, (uintptr) (tskid))
#define LOG_GETTST_LEAVE(ercd, p_state)	trace_3(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETTASKSTATE, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_state)) : (0U)))

/*
 * イベント機能
 */
#define LOG_SETEVT_ENTER(tskid, mask)			trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_SETEVENT, (uintptr) (tskid), (uintptr) (mask))
#define LOG_SETEVT_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_SETEVENT, (uintptr) (ercd))
#define LOG_CLREVT_ENTER(p_runtsk, mask)		trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CLEAREVENT, (uintptr) (p_runtsk), (uintptr) (mask))
#define LOG_CLREVT_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CLEAREVENT, (uintptr) (ercd))
#define LOG_GETEVT_ENTER(tskid)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETEVENT, (uintptr) (tskid))
#define LOG_GETEVT_LEAVE(ercd, tskid, p_mask)	trace_4(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETEVENT, (uintptr) (ercd), (uintptr) (tskid), (((ercd) == E_OK) ? (uintptr) (*(p_mask)) : (0U)))
#define LOG_WAIEVT_ENTER(p_runtsk, mask)		trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_WAITEVENT, (uintptr) (p_runtsk), (uintptr) (mask))
#define LOG_WAIEVT_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_WAITEVENT, (uintptr) (ercd))

/*
 * リソース機能
 */
#define LOG_GETRES_ENTER(resid)	trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETRESOURCE, (uintptr) (resid))
#define LOG_GETRES_LEAVE(ercd)	trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETRESOURCE, (uintptr) (ercd))
#define LOG_RELRES_ENTER(resid)	trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_RELEASERESOURCE, (uintptr) (resid))
#define LOG_RELRES_LEAVE(ercd)	trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_RELEASERESOURCE, (uintptr) (ercd))

/*
 * アラーム機能
 */
#define LOG_GETALB_ENTER(almid)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETALARMBASE, (uintptr) (almid))
#define LOG_GETALB_LEAVE(ercd, info)			trace_5(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETALARMBASE, (uintptr) (ercd), (((ercd) == E_OK) ? ((info)->maxallowedvalue) : (0U)), (((ercd) == E_OK) ? ((info)->ticksperbase) : (0U)), (((ercd) == E_OK) ? ((info)->mincycle) : (0U)))
#define LOG_GETALM_ENTER(almid)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETALARM, (uintptr) (almid))
#define LOG_GETALM_LEAVE(ercd, p_tick)			trace_3(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETALARM, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_tick)) : (0U)))
#define LOG_SETREL_ENTER(almid, incr, cycle)	trace_4(LOG_TYPE_SVC | LOG_ENTER, TFN_SETRELALARM, (uintptr) (almid), (uintptr) (incr), (uintptr) (cycle))
#define LOG_SETREL_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_SETRELALARM, (uintptr) (ercd))
#define LOG_SETABS_ENTER(almid, start, cycle)	trace_4(LOG_TYPE_SVC | LOG_ENTER, TFN_SETABSALARM, (uintptr) (almid), (uintptr) (start), (uintptr) (cycle))
#define LOG_SETABS_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_SETABSALARM, (uintptr) (ercd))
#define LOG_CANALM_ENTER(almid)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CANCELALARM, (uintptr) (almid))
#define LOG_CANALM_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CANCELALARM, (uintptr) (ercd))

/*
 * カウンタ機能
 */
#define LOG_INCCNT_ENTER(cntid)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_INCREMENTCOUNTER, (uintptr) (cntid))
#define LOG_INCCNT_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_INCREMENTCOUNTER, (uintptr) (ercd))
#define LOG_GETCNT_ENTER(cntid)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETCOUNTERVALUE, (uintptr) (cntid))
#define LOG_GETCNT_LEAVE(ercd, p_val)			trace_3(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETCOUNTERVALUE, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_val)) : (0U)))
#define LOG_GETEPS_ENTER(cntid, p_val)			trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_GETELAPSEDVALUE, (uintptr) (cntid), (((p_val) == NULL) ? (0U) : *(p_val)))
#define LOG_GETEPS_LEAVE(ercd, p_val, p_eval)	trace_4(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETELAPSEDVALUE, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_val)) : (0U)), (((ercd) == E_OK) ? (uintptr) (*(p_eval)) : (0U)))

/*
 * スケジュールテーブル機能
 */
#define LOG_STASCHTBLREL_ENTER(schtblid, offset)	trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_STARTSCHEDULETABLEREL, (uintptr) (schtblid), (uintptr) (offset))
#define LOG_STASCHTBLREL_LEAVE(ercd)				trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_STARTSCHEDULETABLEREL, (uintptr) (ercd))
#define LOG_STASCHTBLABS_ENTER(schtblid, start)		trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_STARTSCHEDULETABLEABS, (uintptr) (schtblid), (uintptr) (start))
#define LOG_STASCHTBLABS_LEAVE(ercd)				trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_STARTSCHEDULETABLEABS, (uintptr) (ercd))
#define LOG_STPSCHTBL_ENTER(schtblid)				trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_STOPSCHEDULETABLE, (uintptr) (schtblid))
#define LOG_STPSCHTBL_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_STOPSCHEDULETABLE, (uintptr) (ercd))
#define LOG_NXTSCHTBL_ENTER(from, to)				trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_NEXTSCHEDULETABLE, (uintptr) (from), (uintptr) (to))
#define LOG_NXTSCHTBL_LEAVE(ercd)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_NEXTSCHEDULETABLE, (uintptr) (ercd))
#define LOG_GETSCHTBLST_ENTER(schtblid)				trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETSCHEDULETABLESTATUS, (uintptr) (schtblid))
#define LOG_GETSCHTBLST_LEAVE(ercd, p_status)		trace_3(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETSCHEDULETABLESTATUS, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_status)) : (0U)))

/*
 *  割込み管理機能
 */
#define LOG_DISINT_ENTER()			trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_DISABLEALLINTERRUPTS)
#define LOG_DISINT_LEAVE()			trace_1(LOG_TYPE_SVC | LOG_LEAVE, TFN_DISABLEALLINTERRUPTS)
#define LOG_ENAINT_ENTER()			trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_ENABLEALLINTERRUPTS)
#define LOG_ENAINT_LEAVE()			trace_1(LOG_TYPE_SVC | LOG_LEAVE, TFN_ENABLEALLINTERRUPTS)
#define LOG_SUSALL_ENTER()			trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_SUSPENDALLINTERRUPTS)
#define LOG_SUSALL_LEAVE()			trace_1(LOG_TYPE_SVC | LOG_LEAVE, TFN_SUSPENDALLINTERRUPTS)
#define LOG_RSMALL_ENTER()			trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_RESUMEALLINTERRUPTS)
#define LOG_RSMALL_LEAVE()			trace_1(LOG_TYPE_SVC | LOG_LEAVE, TFN_RESUMEALLINTERRUPTS)
#define LOG_SUSOSI_ENTER()			trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_SUSPENDOSINTERRUPTS)
#define LOG_SUSOSI_LEAVE()			trace_1(LOG_TYPE_SVC | LOG_LEAVE, TFN_SUSPENDOSINTERRUPTS)
#define LOG_RSMOSI_ENTER()			trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_RESUMEOSINTERRUPTS)
#define LOG_RSMOSI_LEAVE()			trace_1(LOG_TYPE_SVC | LOG_LEAVE, TFN_RESUMEOSINTERRUPTS)
#define LOG_GETISRID_ENTER()		trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_GETISRID)
#define LOG_GETISRID_LEAVE(isrid)	trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETISRID, (uintptr) (isrid))
#define LOG_DISINTSRC_ENTER(isrid)	trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_DISABLEINTERRUPTSOURCE, (uintptr) (isrid))
#define LOG_DISINTSRC_LEAVE(ercd)	trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_DISABLEINTERRUPTSOURCE, (uintptr) (ercd))
#define LOG_ENAINTSRC_ENTER(isrid)	trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_ENABLEINTERRUPTSOURCE, (uintptr) (isrid))
#define LOG_ENAINTSRC_LEAVE(ercd)	trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_ENABLEINTERRUPTSOURCE, (uintptr) (ercd))

/*
 *  カーネルの初期化と終了処理
 */
#define LOG_STAOS_ENTER(mode)
#define LOG_STAOS_LEAVE()
#define LOG_STUTOS_ENTER(ercd)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_SHUTDOWNOS, (uintptr) (ercd))
#define LOG_STUTOS_LEAVE()
#define LOG_GETAAM_ENTER()						trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_GETACTIVEAPPLICATIONMODE)
#define LOG_GETAAM_LEAVE(mode)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETACTIVEAPPLICATIONMODE, (uintptr) (mode))
#define LOG_STAHOOK_ENTER()						trace_0(LOG_TYPE_STAHOOK | LOG_ENTER)
#define LOG_STAHOOK_LEAVE()						trace_0(LOG_TYPE_STAHOOK | LOG_LEAVE)
#define LOG_ERRHOOK_ENTER(ercd)					trace_1(LOG_TYPE_ERRHOOK | LOG_ENTER, (uintptr) (ercd))
#define LOG_ERRHOOK_LEAVE()						trace_0(LOG_TYPE_ERRHOOK | LOG_LEAVE)
#define LOG_PROHOOK_ENTER(ercd)					trace_1(LOG_TYPE_PROHOOK | LOG_ENTER, (uintptr) (ercd))
#define LOG_PROHOOK_LEAVE(pret)					trace_1(LOG_TYPE_PROHOOK | LOG_LEAVE, (uintptr) (pret))
#define LOG_SHUTHOOK_ENTER(ercd)				trace_1(LOG_TYPE_SHUTHOOK | LOG_ENTER, (uintptr) (ercd))
#define LOG_SHUTHOOK_LEAVE()					trace_0(LOG_TYPE_SHUTHOOK | LOG_LEAVE)

/*
 *  システムログ機能
 */
#define LOG_SYSLOG_WRI_LOG_LEAVE(ercd)
#define LOG_SYSLOG_REA_LOG_ENTER(p_syslog)
#define LOG_SYSLOG_REA_LOG_LEAVE(ercd, p_syslog)
#define LOG_SYSLOG_MSK_LOG_ENTER(lowmask)
#define LOG_SYSLOG_MSK_LOG_LEAVE(ercd)
#define LOG_SYSLOG_REF_LOG_ENTER(pk_rlog)
#define LOG_SYSLOG_REF_LOG_LEAVE(pk_rlog)

/*
 *  OSAP機能
 */
#define LOG_GETOSAPID_ENTER()							trace_1(LOG_TYPE_SVC | LOG_ENTER, TFN_GETAPPLICATIONID)
#define LOG_GETOSAPID_LEAVE(id)							trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETAPPLICATIONID, (uintptr) (id))
#define LOG_GETAST_ENTER(id)							trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_GETAPPLICATIONSTATE, (uintptr) (id))
#define LOG_GETAST_LEAVE(ercd, p_value)					trace_3(LOG_TYPE_SVC | LOG_LEAVE, TFN_GETAPPLICATIONSTATE, (uintptr) (ercd), (((ercd) == E_OK) ? (uintptr) (*(p_value)) : (0U)))
#define LOG_CALTFN_ENTER(index)							trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CALLTRUSTEDFUNCTION, (uintptr) (index))
#define LOG_CALTFN_LEAVE(ercd)							trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CALLTRUSTEDFUNCTION, (uintptr) (ercd))
#define LOG_TFN_ENTER(index)							trace_1(LOG_TYPE_TFN | LOG_ENTER, (uintptr) (index))
#define LOG_TFN_LEAVE(index, ercd)						trace_2(LOG_TYPE_TFN | LOG_LEAVE, (uintptr) (index), (uintptr) (ercd))
#define LOG_CHKTSKACS_ENTER(ApplID, TaskID)				trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKTASKACCESS, (uintptr) (ApplID), (uintptr) (TaskID))
#define LOG_CHKTSKACS_LEAVE(access)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKTASKACCESS, (uintptr) (access))
#define LOG_CHKISRACS_ENTER(ApplID, ISRID)				trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKISRACCESS, (uintptr) (ApplID), (uintptr) (ISRID))
#define LOG_CHKISRACS_LEAVE(access)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKISRACCESS, (uintptr) (access))
#define LOG_CHKALMACS_ENTER(ApplID, AlarmID)			trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKALARMACCESS, (uintptr) (ApplID), (uintptr) (AlarmID))
#define LOG_CHKALMACS_LEAVE(access)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKALARMACCESS, (uintptr) (access))
#define LOG_CHKRESACS_ENTER(ApplID, ResID)				trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKRESOURCEACCESS, (uintptr) (ApplID), (uintptr) (ResID))
#define LOG_CHKRESACS_LEAVE(access)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKRESOURCEACCESS, (uintptr) (access))
#define LOG_CHKCNTACS_ENTER(ApplID, CounterID)			trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKCOUNTERACCESS, (uintptr) (ApplID), (uintptr) (CounterID))
#define LOG_CHKCNTACS_LEAVE(access)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKCOUNTERACCESS, (uintptr) (access))
#define LOG_CHKSCHTBLACS_ENTER(ApplID, ScheduleTableID)	trace_3(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKSCHEDULETABLEACCESS, (uintptr) (ApplID), (uintptr) (ScheduleTableID))
#define LOG_CHKSCHTBLACS_LEAVE(access)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKSCHEDULETABLEACCESS, (uintptr) (access))
#define LOG_CHKTSKOWN_ENTER(TaskID)						trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKTASKOWNERSHIP, (uintptr) (TaskID))
#define LOG_CHKTSKOWN_LEAVE(owner)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKTASKOWNERSHIP, (uintptr) (owner))
#define LOG_CHKISROWN_ENTER(ISRID)						trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKISROWNERSHIP, (uintptr) (ISRID))
#define LOG_CHKISROWN_LEAVE(owner)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKISROWNERSHIP, (uintptr) (owner))
#define LOG_CHKALMOWN_ENTER(AlarmID)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKALARMOWNERSHIP, (uintptr) (AlarmID))
#define LOG_CHKALMOWN_LEAVE(owner)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKALARMOWNERSHIP, (uintptr) (owner))
#define LOG_CHKCNTOWN_ENTER(CounterID)					trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKCOUNTEROWNERSHIP, (uintptr) (CounterID))
#define LOG_CHKCNTOWN_LEAVE(owner)						trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKCOUNTEROWNERSHIP, (uintptr) (owner))
#define LOG_CHKSCHTBLOWN_ENTER(ScheduleTableID)			trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKSCHEDULETABLEOWNERSHIP, (uintptr) (ScheduleTableID))
#define LOG_CHKSCHTBLOWN_LEAVE(owner)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKSCHEDULETABLEOWNERSHIP, (uintptr) (owner))

/*
 * IOC機能
 */
#define LOG_IOCSEND_ENTER(senderid)		trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_IOC_SEND_GENERIC, (uintptr) (senderid))
#define LOG_IOCSEND_LEAVE(ercd)			trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_IOC_SEND_GENERIC, (uintptr) (ercd))
#define LOG_IOCWRITE_ENTER(senderid)	trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_IOC_WRITE_GENERIC, (uintptr) (senderid))
#define LOG_IOCWRITE_LEAVE(ercd)		trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_IOC_WRITE_GENERIC, (uintptr) (ercd))
#define LOG_IOCRECEIVE_ENTER(iocid)		trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_IOC_RECEIVE_GENERIC, (uintptr) (iocid))
#define LOG_IOCRECEIVE_LEAVE(ercd)		trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_IOC_RECEIVE_GENERIC, (uintptr) (ercd))
#define LOG_IOCREAD_ENTER(iocid)		trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_IOC_READ_GENERIC, (uintptr) (iocid))
#define LOG_IOCREAD_LEAVE(ercd)			trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_IOC_READ_GENERIC, (uintptr) (ercd))
#define LOG_IOCEMPTYQUEUE_ENTER(iocid)	trace_2(LOG_TYPE_SVC | LOG_ENTER, TFN_IOC_EMPTY_QUEUE_GENERIC, (uintptr) (iocid))
#define LOG_IOCEMPTUQUEUE_LEAVE(ercd)	trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_IOC_EMPTY_QUEUE_GENERIC, (uintptr) (ercd))

/*
 *  メモリアクセス関連機能
 */
#define LOG_CHKISRMEMACS_ENTER(ISRID, Address, Size)	trace_4(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKISRMEMORYACCESS, (uintptr) (ISRID), (uintptr) (Address), (uintptr) (Size))
#define LOG_CHKISRMEMACS_LEAVE(access)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKISRMEMORYACCESS, (uintptr) (access))
#define LOG_CHKTSKMEMACS_ENTER(TaskID, Address, Size)	trace_4(LOG_TYPE_SVC | LOG_ENTER, TFN_CHECKTASKMEMORYACCESS, (uintptr) (TaskID), (uintptr) (Address), (uintptr) (Size))
#define LOG_CHKTSKMEMACS_LEAVE(access)					trace_2(LOG_TYPE_SVC | LOG_LEAVE, TFN_CHECKTASKMEMORYACCESS, (uintptr) (access))

#endif /* TOPPERS_ENABLE_TRACE */

#endif /* TOPPERS_TRACE_CONFIG_H */
