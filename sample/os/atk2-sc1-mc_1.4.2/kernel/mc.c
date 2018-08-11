/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2008-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2017 by Witz Corporation
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
 *  $Id: mc.c 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *		マルチコア制御モジュール
 */
#include "kernel_impl.h"
#include "mc.h"
#include "target_ici.h"
#include "task.h"
#include "interrupt.h"
#include "check.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_SHUTALLCORE_ENTER
#define LOG_SHUTALLCORE_ENTER(ercd)
#endif /* LOG_SHUTALLCORE_ENTER */

#ifndef LOG_SHUTALLCORE_LEAVE
#define LOG_SHUTALLCORE_LEAVE()
#endif /* LOG_SHUTALLCORE_LEAVE */


#ifdef TOPPERS_ccb_initialize


#if TTYPE_KLOCK == G_KLOCK

/*
 *  ジャイアントロック
 */
LockType			giant_lock;

#else /* TTYPE_KLOCK == G_KLOCK */

#if TTYPE_SPN == EMULATE_SPN
/*
 *  スピンロックロック
 */
LockType			spn_lock;
#endif /* TTYPE_SPN == EMULATE_SPN */

/*
 *  IOCロック
 */
LockType			ioc_lock;

#endif /* TTYPE_KLOCK == G_KLOCK */

#ifndef OMIT_KER_REQ_ON_ICI
/*
 *  終了処理要求フラグ
 */
volatile boolean	shutdown_reqflg;
#endif /* OMIT_KER_REQ_ON_ICI */

/*
 *  起動しているOS管理のコア数
 *  マスタコアは必ず起動するので1で初期化する
 */
uint32				activated_cores = 1U;

/*
 *  マルチコア管理モジュールの初期化
 */
void
ccb_initialize(void)
{
	uint32		i;
	CCB			*my_p_ccb = get_my_p_ccb();
	CoreIdType	coreid = x_core_id();

	/*
	 *  CCB のcoreid を初期化
	 */
	my_p_ccb->coreid = coreid;

#if TTYPE_KLOCK == G_KLOCK
	/*
	 *  ジャイアントロックの初期化
	 */
	x_initialize_giant_lock(&giant_lock);
#else /* TTYPE_KLOCK != G_KLOCK */
	/*
	 *  タスクロックの初期化
	 */
	x_initialize_tsk_lock(&(my_p_ccb->tsk_lock));

	/*
	 *  カウンタロックの初期化
	 */
	x_initialize_cnt_lock(&(my_p_ccb->cnt_lock));

	if (x_sense_mcore() != FALSE) {
#if TTYPE_SPN == EMULATE_SPN
		/*
		 *  スピンロックロックの初期化
		 */
		x_initialize_spn_lock(&spn_lock);
#endif /* TTYPE_SPN == EMULATE_SPN */

		/*
		 *  IOCロックの初期化
		 */
		x_initialize_ioc_lock(&ioc_lock);
	}

#endif /* TTYPE_KLOCK == C_KLOCK */

	/*
	 *  タスク管理関係の初期化
	 */
	my_p_ccb->p_runtsk = my_p_ccb->p_schedtsk = NULL;
	my_p_ccb->nextpri = TPRI_MINTASK;

	for (i = 0U; i < TNUM_TPRI; i++) {
		queue_initialize(&(my_p_ccb->ready_queue[i]));
	}

	my_p_ccb->ready_primap = 0U;

	/*
	 *  割込み管理関係の初期化
	 */
	my_p_ccb->p_runisr = NULL;

	my_p_ccb->sus_all_cnt = 0U;
	my_p_ccb->wrap_sus_all_cnt = 0U;
	my_p_ccb->sus_os_cnt = 0U;
	my_p_ccb->wrap_sus_os_cnt = 0U;

#ifndef OMIT_KER_REQ_ON_ICI
	if (x_sense_mcore() != FALSE) {
		shutdown_reqflg = FALSE;
	}
#endif /*  OMIT_KER_REQ_ON_ICI */

	/*
	 *  ユーザ定義コア間割込み要求の初期化
	 */
	my_p_ccb->ici_request_map = 0U;

	/*
	 *  ディスパッチ用コア間割込み要求フラグの初期化
	 */
	my_p_ccb->ici_disreqflg = FALSE;

	/*
	 *  OSアプリケーション管理関係の初期化
	 */
	my_p_ccb->p_currentosap = NULL;

	/*
	 *  プロテクションフック用スピンロック管理関係の初期化
	 */
	my_p_ccb->p_protectspncb = NULL;
}

#endif /* TOPPERS_ccb_initialize */

/*
 *  コア間割込みハンドラ
 */
#ifdef TOPPERS_ici_handler_main

#ifdef CFG_USE_STACKMONITORING
/*
 *  ICISR処理毎に実施するスタックモニタリング処理
 */
LOCAL_INLINE void
ici_stk_monitoring(void)
{
	StackType *ostk_top;

	ostk_top = ostk_table[x_core_id()];
	if (ostk_top[0] != STACK_MAGIC_NUMBER) {
		stack_monitoring_error_isr();
	}
}
#endif /* CFG_USE_STACKMONITORING */

void
ici_handler_main(void)
{
	CCB				*my_p_ccb;
	uint32			reqmap;
	uint16			reqmap_half;
	uint32			icino;
	ISRCB			*p_runisr_saved;
	const OSAPINIB	*p_currentosap_saved;
	const ICIINIB	*p_iciinib;

	my_p_ccb = get_my_p_ccb();

	target_ici_clear();

#ifndef OMIT_KER_REQ_ON_ICI
	/*
	 *  終了処理要求をチェック
	 */
	if (shutdown_reqflg != FALSE) {
		internal_shutdownallcores(E_OS_SHUTDOWN_OTHER_CORE);
	}
#endif /* OMIT_KER_REQ_ON_ICI */

	x_nested_lock_os_int();
	acquire_tsk_lock(my_p_ccb);

	reqmap = my_p_ccb->ici_request_map & my_p_ccb->ici_bit_mask;
	while (reqmap != 0U) {
		/* コア間割込みの要因を検索(最下位ビット優先) */
		reqmap_half = (uint16) (reqmap & 0x0000ffffU);
		if (reqmap_half != 0U) {
			icino = (uint32) bitmap_search(reqmap_half);
		}
		else {
			reqmap_half = (uint16) ((reqmap & 0xffff0000U) >> 16U);
			icino = (uint32) bitmap_search(reqmap_half) + 16U;
		}

		if (icino < tnum_ici_of_core[my_p_ccb->coreid]) {
			/* 個別割込み禁止しているかチエック */
			p_iciinib = p_iciinb_table[my_p_ccb->coreid];
			if ((my_p_ccb->ici_bit_mask & p_iciinib[icino].ici_bit_ptn) != 0U) {
				/* 探索された要因の要求ビットを落して，ICISRハンドラを呼び出す */
				my_p_ccb->ici_request_map &= ~(p_iciinib[icino].ici_bit_ptn);

				/*
				 *  ユーザ定義の要因だった場合，ICISRハンドラの呼び出し
				 *  ICISRハンドラ実行中にこのICISRの所属OSAPをp_currentosapに設定する
				 */
				p_runisr_saved = my_p_ccb->p_runisr;
				my_p_ccb->p_runisr = get_isrcb(((p_iciinb_table[my_p_ccb->coreid])[icino]).iciid);
				p_currentosap_saved = my_p_ccb->p_currentosap;
				my_p_ccb->p_currentosap = my_p_ccb->p_runisr->p_isrinib->p_osapinib;
				release_tsk_lock(my_p_ccb);
				x_nested_unlock_os_int();

				(((p_iciinb_table[my_p_ccb->coreid])[icino]).ici_routing)();

	#ifdef CFG_USE_STACKMONITORING
				/* ICISR処理毎にスタックモニタリングを実施 */
				ici_stk_monitoring();
	#endif /* CFG_USE_STACKMONITORING */

				/* ICISR不正終了チェック */
				exit_isr2();

				x_nested_lock_os_int();
				acquire_tsk_lock(my_p_ccb);
				my_p_ccb->p_currentosap = p_currentosap_saved;
				my_p_ccb->p_runisr = p_runisr_saved;
			}
		}
		else {
			/*
			 *  ユーザ定義以外のICISRの場合
			 *  ここに来ることがない
			 */
			/* 探索された要因の要求ビットを落とす */
			my_p_ccb->ici_request_map &= ~((uint32) 0x00000001U << icino);
		}
		reqmap = my_p_ccb->ici_request_map & my_p_ccb->ici_bit_mask;
	}

	/*
	 *  ディスパッチ用コア間割込み要求フラグをクリア
	 */
	my_p_ccb->ici_disreqflg = FALSE;

	release_tsk_lock(my_p_ccb);
	x_nested_unlock_os_int();
}

#endif /* TOPPERS_ici_handler_main */

/*
 *  ディスパッチ要求
 *  対象が他コアの場合はコア間割り込みを発行する
 *
 *  ※ タスクロック獲得中に呼び出されることを前提とする．※
 */
#ifdef TOPPERS_dispatch_request

boolean
dispatch_request(CCB *p_ccb)
{
	CCB *my_p_ccb;

	my_p_ccb = get_my_p_ccb();

	if (p_ccb == my_p_ccb) {
		return(TRUE);
	}
	else {
		/* ディスパッチコア間割込み発行した場合，再発行しない */
		if (p_ccb->ici_disreqflg == FALSE) {
			p_ccb->ici_disreqflg = TRUE;
			target_ici_dispreq(p_ccb);
		}
		return(FALSE);
	}
}

#endif /* TOPPERS_dispatch_request */

#ifndef OMIT_KER_REQ_ON_ICI
#ifdef TOPPERS_shutdown_request

/*
 *  終了処理要求
 */
void
shutdown_request(void)
{
	CoreIdType	coreid;
	CoreIdType	my_coreid = x_core_id();
	CCB			*p_ccb;

	/* すでに要求が出ていればリターン */
	if (shutdown_reqflg == FALSE) {
		shutdown_reqflg = TRUE;
		for (coreid = 0U; coreid < TotalNumberOfCores; coreid++) {
			p_ccb = get_p_ccb(coreid);
			if ((p_ccb->kerflg != FALSE) && (coreid != my_coreid)) {
				target_ici_raise(coreid);
			}
		}
	}
}

#endif /* TOPPERS_shutdown_request */
#endif /* OMIT_KER_REQ_ON_ICI */

#ifndef OMIT_BARRIER_SYNC
#ifdef TOPPERS_barrier_sync

#ifndef TNUM_BUSY_LOOP
#define TNUM_BUSY_LOOP	100U
#endif /* TNUM_BUSY_LOOP */

/*
 *  バリア同期内部で用いる待ち処理
 */
LOCAL_INLINE void
busy_wait(void)
{
	volatile uint8 i;
	for (i = 0U; i < TNUM_BUSY_LOOP; i++) {
	}
}

/*
 *  バリア同期
 */
void
barrier_sync(uint8 phase, boolean check_shutdown)
{
	CoreIdType				i;
	CoreIdType				flag;
	static volatile uint8	sys_start;
	static volatile uint8	core_init[TotalNumberOfCores] = {
		0U
	};

	core_init[x_core_id()] = phase;

	if (x_sense_mcore() != FALSE) {
		do {
			flag = 0U;
			for (i = 0U; i < TotalNumberOfCores; i++) {
				if (core_init[i] == phase) {
					flag++;
				}
			}
			busy_wait();
			if ((check_shutdown != FALSE) && (shutdown_reqflg != FALSE)) {
				/*
				 *  internal_shutdownallcoresを呼ぶ前にOS割込み禁止状態へ
				 *  全割込み禁止状態解除
				 */
				x_nested_lock_os_int();
				x_unlock_all_int();
				internal_shutdownallcores(E_OS_SHUTDOWN_OTHER_CORE);
			}
		} while (flag < activated_cores);
		sys_start = phase;
	}
	else {
		while (sys_start != phase) {
			busy_wait();
			if ((check_shutdown != FALSE) && (shutdown_reqflg != FALSE)) {
				/*
				 *  internal_shutdownallcoresを呼ぶ前にOS割込み禁止状態へ
				 *  全割込み禁止状態解除
				 */
				x_nested_lock_os_int();
				x_unlock_all_int();
				internal_shutdownallcores(E_OS_SHUTDOWN_OTHER_CORE);
			}
		}
	}
}

#endif /* TOPPERS_barrier_sync */
#endif /* OMIT_BARRIER_SYNC */

/*
 *  OS内部からのShutdownAllCoresの呼び出し
 */
#ifdef TOPPERS_internal_shutdownallcores

void
internal_shutdownallcores(StatusType ercd)
{
	LOG_SHUTALLCORE_ENTER(ercd);
	x_nested_lock_os_int();

	shutdown_request();

#ifdef CFG_USE_SHUTDOWNHOOK
	call_shutdownhook(ercd);
#endif /* CFG_USE_SHUTDOWNHOOK */

	/* 各モジュールの終了処理 */
	object_terminate();

	x_lock_all_int();
	LOG_SHUTALLCORE_LEAVE();

	/* ターゲット依存の終了処理 */
	target_exit();

	/*
	 * ターゲット依存部から処理が返ってきた場合，
	 * 無限ループを行う
	 */
	while (1) {
	}

}

#endif /* TOPPERS_internal_shutdownallcores */


#if TTYPE_KLOCK != G_KLOCK
/*
 *  コアロックのロック取得・解放関数
 */
/*
 *		タスクコンテキスト用のタスクロック取得関数
 */

/*
 *  任意タスク（自タスクも含む）のタスクロックの取得（任意のコンテキスト/1段目）
 */
#ifdef TOPPERS_acquire_tsk_lock

void
acquire_tsk_lock(CCB *p_ccb)
{
	x_acquire_lock(&(p_ccb->tsk_lock));
}

#endif /* TOPPERS_acquire_tsk_lock */

/*
 *  タスクロックの解放（任意のコンテキスト）
 */
#ifdef TOPPERS_release_tsk_lock

void
release_tsk_lock(CCB *p_ccb)
{
	x_release_lock(&(p_ccb->tsk_lock));
}

#endif /* TOPPERS_release_tsk_lock */

/*
 *  タスクロックの解放とディスパッチ
 */
#ifdef TOPPERS_release_tsk_lock_and_dispatch

void
release_tsk_lock_and_dispatch(CCB *p_ccb,  boolean dspreq)
{
	x_release_lock(&(p_ccb->tsk_lock));
	if (dspreq != FALSE) {
		dispatch();
	}
}

#endif /* TOPPERS_release_tsk_lock_and_dispatch */

/*
 *  カウンタロックの取得（任意のコンテキスト）
 */
#ifdef TOPPERS_acquire_cnt_lock

void
acquire_cnt_lock(CCB *p_ccb)
{
	x_acquire_lock(&(p_ccb->cnt_lock));
}

#endif /* TOPPERS_acquire_cnt_lock */

/*
 * カウンタロックの解放（任意のコンテキスト）
 */
#ifdef TOPPERS_release_cnt_lock

void
release_cnt_lock(CCB *p_ccb)
{
	x_release_lock(&(p_ccb->cnt_lock));
}

#endif /* TOPPERS_release_cnt_lock */

/*
 *	スピンロックの取得（任意のコンテキスト）
 */
#ifdef TOPPERS_acquire_spn_lock

void
acquire_spn_lock(LockType *p_spnlock)
{
	x_acquire_lock(p_spnlock);
}

#endif /* TOPPERS_acquire_spn_lock */

/*
 *	スピンロックの解放（任意のコンテキスト）
 */
#ifdef TOPPERS_release_spn_lock

void
release_spn_lock(LockType *p_spnlock)
{
	x_release_lock(p_spnlock);
}

#endif /* TOPPERS_release_spn_lock */

/*
 *	スピンロックの取得試行（任意のコンテキスト）
 */
#ifdef TOPPERS_try_spn_lock

boolean
try_spn_lock(LockType *p_spnlock)
{
	return(x_try_lock(p_spnlock));
}

#endif /* TOPPERS_try_spn_lock */

/*
 *  IOCロックの取得（任意のコンテキスト）
 */
#ifdef TOPPERS_acquire_ioc_lock
void
acquire_ioc_lock(LockType *p_ioclock)
{
	x_acquire_lock(p_ioclock);
}

#endif /* TOPPERS_acquire_ioc_lock */

/*
 *	IOCロックの解放（任意のコンテキスト）
 */
#ifdef TOPPERS_release_ioc_lock
void
release_ioc_lock(LockType *p_ioclock)
{
	x_release_lock(p_ioclock);
}

#endif /* TOPPERS_release_ioc_lock */

#endif /* TTYPE_KLOCK != G_KLOCK */
