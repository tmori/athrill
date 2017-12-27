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
 *  $Id: task.c 425 2015-12-07 08:06:19Z witz-itoyo $
 */

/*
 *		タスク制御モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "interrupt.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_TSKSTAT
#define LOG_TSKSTAT(p_tcb)
#endif /* LOG_TSKSTAT */

/* 内部関数のプロトタイプ宣言 */
LOCAL_INLINE uint16 bitmap_search(uint16 bitmap);
LOCAL_INLINE boolean primap_empty(void);
LOCAL_INLINE PriorityType primap_search(void);
LOCAL_INLINE void primap_set(PriorityType pri);
LOCAL_INLINE void primap_clear(PriorityType pri);

#ifdef TOPPERS_task_initialize

/*
 *  実行状態のタスク
 */
TCB				*p_runtsk;

/*
 *  最高優先順位タスク
 */
TCB				*p_schedtsk;

/*
 *  レディキュー中の最高優先度
 */
PriorityType	nextpri;

/*
 *  レディキュー
 */
QUEUE			ready_queue[TNUM_TPRI];

/*
 *  レディキューサーチのためのビットマップ
 */
uint16			ready_primap;

/*
 *  タスク管理モジュールの初期化
 */
void
task_initialize(void)
{
	TaskType	i;
	TCB			*p_tcb;

	p_runtsk = NULL;
	p_schedtsk = NULL;

	for (i = 0U; i < TNUM_TPRI; i++) {
		queue_initialize(&(ready_queue[i]));
	}
	nextpri = TPRI_MINTASK;
	ready_primap = 0U;

	for (i = 0U; i < tnum_task; i++) {
		p_tcb = &(tcb_table[i]);
		p_tcb->p_tinib = &(tinib_table[i]);
		p_tcb->actcnt = 0U;
		p_tcb->tstat = SUSPENDED;
#ifdef CFG_USE_PROTECTIONHOOK
		p_tcb->calltfn = FALSE;
#endif /* CFG_USE_PROTECTIONHOOK */
		if ((p_tcb->p_tinib->autoact & ((AppModeType) 1 << appmodeid)) != APPMODE_NONE) {
			(void) make_active(p_tcb);
		}
	}
}

#endif /* TOPPERS_task_initialize */

/*
 *  ビットマップサーチ関数
 *
 *  bitmap内の1のビットの内，最も下位（右）のものをサーチし，そのビッ
 *  ト番号を返す
 *  ビット番号は，最下位ビットを0とする．bitmapに0を指定
 *  してはならない．この関数では，bitmapが16ビットであることを仮定し，
 *  uint16型としている
 *
 *  ビットサーチ命令を持つプロセッサでは，ビットサーチ命令を使うように
 *  書き直した方が効率が良い場合がある
 *  このような場合には，ターゲット依存部でビットサーチ命令を使った
 *  bitmap_searchを定義し，OMIT_BITMAP_SEARCHをマクロ定義すればよい
 *  また，ビットサーチ命令のサーチ方向が逆などの理由で優先度とビット
 *  との対応を変更したい場合には，PRIMAP_BITをマクロ定義すればよい
 *
 *  また，標準ライブラリにffsがあるなら，次のように定義して標準ライブ
 *  ラリを使った方が効率が良い可能性もある
 *		#define	bitmap_search(bitmap) (ffs(bitmap) - 1)
 */
#ifndef PRIMAP_BIT
#define PRIMAP_BIT(pri)		((uint16) ((uint16) 1U << (pri)))
#endif /* PRIMAP_BIT */

#ifndef OMIT_BITMAP_SEARCH

LOCAL_INLINE uint16
bitmap_search(uint16 bitmap)
{
	/*
	 *  ビットマップサーチ関数用テーブル
	 */
	const uint8	bitmap_search_table[BITMAP_NUM] = {
		0U, 1U, 0U, 2U, 0U, 1U, 0U,
		3U, 0U, 1U, 0U, 2U, 0U, 1U, 0U
	};

	uint16		n = 0U;

	ASSERT(bitmap != 0U);
	if ((bitmap & 0x00ffU) == 0U) {
		bitmap >>= 8U;
		n += 8U;
	}
	if ((bitmap & 0x0fU) == 0U) {
		bitmap >>= 4U;
		n += 4U;
	}
	return(n + bitmap_search_table[(bitmap & 0x000fU) - 1U]);
}

#endif /* OMIT_BITMAP_SEARCH */

/*
 *  優先度ビットマップが空かのチェック
 */
LOCAL_INLINE boolean
primap_empty(void)
{
	return(ready_primap == 0U);
}

/*
 *  優先度ビットマップのサーチ
 */
LOCAL_INLINE PriorityType
primap_search(void)
{
	return((PriorityType) bitmap_search(ready_primap));
}

/*
 *  優先度ビットマップのセット
 */
LOCAL_INLINE void
primap_set(PriorityType pri)
{
	ready_primap |= PRIMAP_BIT(pri);
}

/*
 *  優先度ビットマップのクリア
 */
LOCAL_INLINE void
primap_clear(PriorityType pri)
{
	ready_primap &= (uint16) ~PRIMAP_BIT(pri);
}

/*
 *  最高優先順位タスクのサーチ
 */
#ifdef TOPPERS_search_schedtsk

void
search_schedtsk(void)
{
	if (primap_empty() != FALSE) {
		p_schedtsk = NULL;
	}
	else {
		p_schedtsk = (TCB *) (queue_delete_next(&(ready_queue[nextpri])));
		if (queue_empty(&(ready_queue[nextpri])) != FALSE) {
			primap_clear(nextpri);
			nextpri = (primap_empty() != FALSE) ?
					  TPRI_MINTASK : primap_search();
		}
	}
}

#endif /* TOPPERS_search_schedtsk */

/*
 *  実行できる状態への移行
 */
#ifdef TOPPERS_make_runnable

boolean
make_runnable(TCB *p_tcb)
{
	PriorityType	pri, schedpri;
	boolean			is_next_schedtsk = TRUE;

	p_tcb->tstat = READY;
	LOG_TSKSTAT(p_tcb);
	if (p_schedtsk != NULL) {
		pri = p_tcb->curpri;
		schedpri = p_schedtsk->curpri;
		if (pri >= schedpri) {
			/*
			 *  schedtsk の方が優先度が高い場合，p_tcb をレ
			 *  ディキューの最後に入れる
			 */
			queue_insert_prev(&(ready_queue[pri]), &(p_tcb->task_queue));
			primap_set(pri);
			if (pri < nextpri) {
				nextpri = pri;
			}
			is_next_schedtsk = FALSE;
		}
		else {
			/*
			 *  p_tcb の方が優先度が高い場合，schedtsk をレディキュー
			 *  の先頭に入れ，p_tcb を新しい schedtsk とする
			 */
			queue_insert_next(&(ready_queue[schedpri]), &(p_schedtsk->task_queue));
			primap_set(schedpri);
			nextpri = schedpri;
		}
	}

	if (is_next_schedtsk != FALSE) {
		p_schedtsk = p_tcb;
	}
	return(is_next_schedtsk);
}

#endif /* TOPPERS_make_runnable */

/*
 *  実行できる状態から他の状態への遷移
 *
 *  SC1-MCでmake_non_runnableが実装されるため
 *  SC1にもmake_non_runableを実装し，SC1もSC1-MCの関数構成に合せる
 *  （SC1-MCでは，p_runtskとp_schedtskが一致していることを確認する処理を入れる必要があるが，
 *  SC1では，search_schedtskのみ呼び出す処理とする）
 */
#ifdef TOPPERS_make_non_runnable

void
make_non_runnable(void)
{
	search_schedtsk();
}

#endif /* TOPPERS_make_non_runnable */

/*
 *  タスクの起動
 *
 *  TerminateTask や ChainTask の中で，自タスクに対して make_active を
 *  呼ぶ場合があるので注意する
 */
#ifdef TOPPERS_make_active

boolean
make_active(TCB *p_tcb)
{
	p_tcb->curpri = p_tcb->p_tinib->inipri;
	if (TSKID(p_tcb) < tnum_exttask) {
		p_tcb->curevt = EVTMASK_NONE;
		p_tcb->waievt = EVTMASK_NONE;
	}
	p_tcb->p_lastrescb = NULL;
	p_tcb->p_lastcntcb = NULL;

	activate_context(p_tcb);
	return(make_runnable(p_tcb));
}

#endif /* TOPPERS_make_active */

/*
 *  タスクのプリエンプト
 */
#ifdef TOPPERS_preempt

void
preempt(void)
{
	PriorityType pri;

	ASSERT(p_runtsk == p_schedtsk);
	pri = p_runtsk->curpri;
	queue_insert_next(&(ready_queue[pri]), &(p_runtsk->task_queue));
	primap_set(pri);
	search_schedtsk();
}

#endif /* TOPPERS_preempt */

/*
 *  実行中のタスクをSUSPENDED状態にする
 *  リソース解放した状態で呼出す必要がある
 */
#ifdef TOPPERS_suspend

void
suspend(void)
{
	p_runtsk->tstat = SUSPENDED;
	LOG_TSKSTAT(p_runtsk);
	make_non_runnable();
	if (p_runtsk->actcnt > 0U) {
		p_runtsk->actcnt -= 1U;
		(void) make_active(p_runtsk);
	}
}

#endif /* TOPPERS_suspend */

/*
 *  タスクの全リソース返却
 */
#ifdef TOPPERS_release_taskresources

void
release_taskresources(TCB *p_tcb)
{
	if (p_tcb->p_lastrescb != NULL) {
		if (p_tcb->curpri <= TPRI_MINISR) {
			/* リソースを全部解放すれば割込み許可になる */
			x_set_ipm((PriorityType) TIPM_ENAALL);
		}
		/* リソースを全部解放すれば実行中優先度に戻る */
		p_tcb->curpri = p_tcb->p_tinib->exepri;

		/* OS割込み禁止状態以上で来る p_tcb->p_lastrescb != NULL */
		do {
			p_tcb->p_lastrescb->lockflg = FALSE;
			p_tcb->p_lastrescb = p_tcb->p_lastrescb->p_prevrescb;
		} while (p_tcb->p_lastrescb != NULL);
	}
}

#endif /* TOPPERS_release_taskresources */

/*
 *  タスクのカウンタの状態をすべてCS_NULLに戻す
 */
#ifdef TOPPERS_cancel_taskcounters

void
cancel_taskcounters(TCB *p_tcb)
{
	if (p_tcb->p_lastcntcb != NULL) {

		/* OS割込み禁止状態以上で来る */
		do {
			p_tcb->p_lastcntcb->cstat = CS_NULL;
			p_tcb->p_lastcntcb = p_tcb->p_lastcntcb->p_prevcntcb;
		} while (p_tcb->p_lastcntcb != NULL);
	}
}
#endif /* TOPPERS_cancel_taskcounters */

/*
 *  タスクの不正終了時の保護
 *  TerminateTask()，ChainTask()なしでの自タスクの終了
 * （タスクの関数からリターン）した場合の処理
 */
#ifdef TOPPERS_exit_task

void
exit_task(void)
{
	x_nested_lock_os_int();

	/* 割込み禁止状態の場合は割込み禁止を解除する */
	release_interrupts(OSServiceId_Invalid);

	/* リソース確保したままの場合はリソースを解放する */
	release_taskresources(p_runtsk);

#ifdef CFG_USE_ERRORHOOK
	/* エラーフックを呼ぶ */
	call_errorhook(E_OS_MISSINGEND, OSServiceId_TaskMissingEnd);
#endif /* CFG_USE_ERRORHOOK */

	suspend();

	/* ポストタスクフックが有効な場合はポストタスクフックが呼ばれる */
	exit_and_dispatch();
}

#endif /* TOPPERS_exit_task */

#ifdef TOPPERS_remove_task_from_queue

/*
 *  レディキューからタスクを削除する
 */
void
remove_task_from_queue(TCB *p_tcb, PriorityType remove_task_pri)
{
	queue_delete(&(p_tcb->task_queue));
	if (queue_empty(&(ready_queue[remove_task_pri])) != FALSE) {
		primap_clear(remove_task_pri);
		nextpri = (primap_empty() != FALSE) ?	TPRI_MINTASK : primap_search();
	}
}

#endif /* TOPPERS_remove_task_from_queue */

/*
 *  自タスクの強制終了
 *    ProtectionHookからのみ呼出される
 *      リソースの解放，割込み禁止の解除を行った後
 *       TerminateTask相当の処理を行う
 *      強制タスク終了の場合は，PostTaskHookを呼出さないのでTerminateTask
 *      とは異なる処理となる
 *  本関数はOS割込み禁止状態で呼出されることを前提とする
 */
#ifdef TOPPERS_force_terminate_task

void
force_terminate_task(TCB *p_tcb)
{
	/*
	 *  割込み禁止を解除する
	 *  エラーフックを呼ばないため，引数にOSServiceId_Invalidをする
	 */
	release_interrupts(OSServiceId_Invalid);

	/* リソース確保したままの場合，リソース解放 */
	release_taskresources(p_tcb);
	/* カウンタの状態を初期化する */
	cancel_taskcounters(p_tcb);

	suspend();

	exit_and_dispatch_nohook();
}

#endif /* TOPPERS_force_terminate_task */

/*
 *  OSAP所属するタスクの強制終了
 */
#ifdef TOPPERS_force_term_osap_task

void
force_term_osap_task(OSAPCB *p_osapcb)
{
	/* OSAPに所属するタスクを強制終了 */
	TaskType		i;
	TCB				*p_tcb;
	PriorityType	remove_task_pri;

	for (i = 0U; i < tnum_task; i++) {
		p_tcb = &tcb_table[i];
		if ((tinib_table[i].p_osapcb == p_osapcb) &&
			(p_tcb != p_runtsk)) {
			/*
			 *  OSAP終了/再起動よりタスクの強制終了
			 */
			p_tcb->actcnt = 0U;
#ifdef CFG_USE_PROTECTIONHOOK
			p_tcb->calltfn = FALSE;
#endif /* CFG_USE_PROTECTIONHOOK */

			if (p_tcb->tstat == READY) {
				/* curpriとinipriが異なる値でレディキューに繋がれている場合への対応 */
				remove_task_pri = p_tcb->curpri;

				/* リソース確保したままの場合，リソース解放 */
				release_taskresources(p_tcb);

				/* カウンタの状態を初期化する */
				cancel_taskcounters(p_tcb);

				/* 対象タスクをSUSPEND状態とし，レディキューから削除する */
				p_tcb->tstat = SUSPENDED;
				remove_task_from_queue(p_tcb, remove_task_pri);
			}
			else {
				p_tcb->tstat = SUSPENDED;
			}

			/*
			 *  ここでC2ISRを受け付ける.
			 *  フックの中からここに来た場合はnested_lock_os_int_cntが2以上なので，
			 *  割込み禁止解除されない.
			 *  SusOS状態でも，割込み優先度マスクは保持されたままなので，
			 *  C2ISRは発生しない.
			 */
			x_nested_unlock_os_int();

			/* ここで割込み禁止とする */
			x_nested_lock_os_int();
		}
	}

	p_tcb = p_runtsk;
	/* OSAPを強制終了するためのリスタートタスクからコールされるため */
	/* リソース解放やカウンタの状態を初期化する必要はなし           */
	/* 対象タスクをSUSPEND状態とする */
	/* p_schedtskの更新はforce_term_osap_mainに戻ってから実施する */
	p_tcb->tstat = SUSPENDED;
}

#endif /* TOPPERS_force_term_osap_task */

/*
 *  p_schedtskをレディキューの先頭に退避
 */
#ifdef TOPPERS_move_schedtsk

void
move_schedtsk(void)
{
	queue_insert_next(&(ready_queue[p_schedtsk->curpri]), &(p_schedtsk->task_queue));
	primap_set(p_schedtsk->curpri);
	nextpri = p_schedtsk->curpri;
}

#endif /* TOPPERS_move_schedtsk */
