/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 * 
 *  Copyright (C) 2005-2016 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: tBitMutex.c 756 2016-10-03 10:47:38Z ertl-hiro $
 */

/*
 *		ミューテックス機能の整合性検査
 */

#include "tBitMutex_tecsgen.h"
#include "kernel/kernel_impl.h"
#include "kernel/task.h"
#include "kernel/mutex.h"
#include "kernel/check.h"

/*
 *  ミューテックスIDからミューテックス管理ブロックを取り出すためのマク
 *  ロ（mutex.cより）
 */
#define INDEX_MTX(mtxid)	((uint_t)((mtxid) - TMIN_MTXID))
#define get_mtxcb(mtxid)	(&(mtxcb_table[INDEX_MTX(mtxid)]))

/*
 *  ミューテックスのプロトコルを判断するマクロ
 */
#define MTXPROTO_MASK			0x03U
#define MTXPROTO(p_mtxinib)		((p_mtxinib)->mtxatr & MTXPROTO_MASK)
#define MTX_CEILING(p_mtxinib)	(MTXPROTO(p_mtxinib) == TA_CEILING)

/*
 *   エラーコードの定義
 */
#define E_SYS_LINENO	ERCD(E_SYS, -(__LINE__))

/*
 *  管理ブロックのアドレスの正当性のチェック
 */
#define VALID_TCB(p_tcb) \
		((((char *) p_tcb) - ((char *) tcb_table)) % sizeof(TCB) == 0 \
			&& TMIN_TSKID <= TSKID(p_tcb) && TSKID(p_tcb) <= tmax_tskid)

#define VALID_MTXCB(p_mtxcb) \
		((((char *) p_mtxcb) - ((char *) mtxcb_table)) % sizeof(MTXCB) == 0 \
			&& TMIN_MTXID <= MTXID(p_mtxcb) && MTXID(p_mtxcb) <= tmax_mtxid)
				
/*
 *  キューのチェックのための関数
 *
 *  p_queueにp_entryが含まれているかを調べる．含まれていればtrue，含ま
 *  れていない場合にはfalseを返す．ダブルリンクの不整合の場合にも，
 *  falseを返す．
 */
static bool_t
in_queue(QUEUE *p_queue, QUEUE *p_entry)
{
	QUEUE	*p_current, *p_next;

	p_current = p_queue->p_next;
	if (p_current->p_prev != p_queue) {
		return(false);					/* ダブルリンクの不整合 */
	}
	while (p_current != p_queue) {
		if (p_current == p_entry) {
			return(true);				/* p_entryが含まれていた */
		}

		/*
		 *  キューの次の要素に進む
		 */
		p_next = p_current->p_next;
		if (p_next->p_prev != p_current) {
			return(false);				 /* ダブルリンクの不整合 */
		}
		p_current = p_next;
	}
	return(false);
}

/*
 *  タスク毎の検査
 */
static ER
bit_mutex_task(ID tskid)
{
	TCB				*p_tcb;
	uint_t			pri;
	MTXCB			*p_mtxcb;
	const MTXINIB	*p_mtxinib;

	if (!VALID_TSKID(tskid)) {
		return(E_ID);
	}
	p_tcb = get_tcb(tskid);
	pri = p_tcb->bpriority;

	/*
	 *  タスクがロックしているミューテックスのキューの検査
	 */
	p_mtxcb = p_tcb->p_lastmtx;
	while (p_mtxcb != NULL) {
		if (!VALID_MTXCB(p_mtxcb)) {
			return(E_SYS_LINENO);
		}
		p_mtxinib = p_mtxcb->p_mtxinib;

		/*
		 *  ミューテックスをロックしているタスクのチェック
		 */
		if (p_mtxcb->p_loctsk != p_tcb) {
			return(E_SYS_LINENO);
		}

		/*
		 *  現在優先度の計算
		 */
		if (MTXPROTO(p_mtxinib)) {
			if (p_mtxinib->ceilpri < pri) {
				pri = p_mtxinib->ceilpri;
			}
		}

		/*
		 *  キューの次の要素に進む
		 */
		p_mtxcb = p_mtxcb->p_prevmtx;
	}

	/*
	 *  現在優先度の検査
	 */
	if (p_tcb->priority != pri) {
		return(E_SYS_LINENO);
	}

	/*
	 *  タスクが待っているミューテックスに関する検査
	 */
	if (TSTAT_WAIT_MTX(p_tcb->tstat)) {
		p_mtxcb = ((WINFO_MTX *)(p_tcb->p_winfo))->p_mtxcb;
		if (!VALID_MTXCB(p_mtxcb)) {
			return(E_SYS_LINENO);
		}
		if (!in_queue(&(p_mtxcb->wait_queue), &(p_tcb->task_queue))) {
			return(E_SYS_LINENO);
		}
	}
	return(E_OK);
}

/*
 *  ミューテックス毎の検査
 */
static ER
bit_mutex_mutex(ID mtxid)
{
	MTXCB			*p_mtxcb, *p_acquired_mtx;
	const MTXINIB	*p_mtxinib;
	TCB				*p_tcb;
	QUEUE			*p_queue, *p_next;
	uint_t			pri;

	if (!VALID_MTXID(mtxid)) {
		return(E_ID);
	}
	p_mtxcb = get_mtxcb(mtxid);
	p_mtxinib = p_mtxcb->p_mtxinib;

	/*
	 *  初期化ブロックへのポインタの検査
	 */
	if (p_mtxinib != &(mtxinib_table[INDEX_MTX(mtxid)])) {
		return(E_SYS_LINENO);
	}

	/*
	 *  ミューテックス待ちキューの検査
	 */
	p_queue = p_mtxcb->wait_queue.p_next;
	if (p_queue->p_prev != &(p_mtxcb->wait_queue)) {
		return(E_SYS_LINENO);
	}
	pri = TMIN_TPRI;
	while (p_queue != &(p_mtxcb->wait_queue)) {
		p_tcb = (TCB *) p_queue;
		if (!VALID_TCB(p_tcb)) {
			return(E_SYS_LINENO);
		}

		/*
		 *  キューがタスク優先度順になっているかの検査
		 */
		if (MTXPROTO(p_mtxinib) != TA_NULL) {
			if (p_tcb->priority < pri) {
				return(E_SYS_LINENO);
			}
		}
		pri = p_tcb->priority;

		/*
		 *  タスク状態の検査
		 *
		 *  ミューテックス待ち状態のタスクの検査は，タスク毎の検査で行っ
		 *  ているため，ここでは行わない．
		 */
		if (!TSTAT_WAIT_MTX(p_tcb->tstat)) {
			return(E_SYS_LINENO);
		}

		/*
		 *  優先度上限の検査
		 */
		if (MTX_CEILING(p_mtxinib)) {
			if (p_tcb->bpriority < p_mtxinib->ceilpri) {
				return(E_SYS_LINENO);
			}
		}

		/*
		 *  キューの次の要素に進む
		 */
		p_next = p_queue->p_next;
		if (p_next->p_prev != p_queue) {
			return(E_SYS_LINENO);
		}
		p_queue = p_next;
	}

	/*
	 *  ミューテックスをロックしているタスクの検査
	 */
	p_tcb = p_mtxcb->p_loctsk;
	if (p_tcb == NULL) {
		/*
		 *  ミューテックスがロックされていない時
		 */
		if (!queue_empty(&(p_mtxcb->wait_queue))) {
			return(E_SYS_LINENO);
		}
	}
	else {
		/*
		 *  ミューテックスがロックされている時
		 *
		 *  ミューテックスをロックしているタスクの検査は，タスク毎の検
		 *  査で行っているため，ここでは行わない．
		 */
		if (!VALID_TCB(p_tcb)) {
			return(E_SYS_LINENO);
		}
		p_acquired_mtx = p_tcb->p_lastmtx;
		while (p_mtxcb != NULL) {
			if (p_mtxcb == p_acquired_mtx) {
				break;
			}
			p_acquired_mtx = p_acquired_mtx->p_prevmtx;
		}
		if (p_mtxcb == NULL) {
			return(E_SYS_LINENO);
		}

		/*
		 *  優先度上限の検査
		 */
		if (MTX_CEILING(p_mtxinib)) {
			if (p_tcb->bpriority < p_mtxinib->ceilpri) {
				return(E_SYS_LINENO);
			}
		}
	}
	return(E_OK);
}

/*
 *  整合性検査関数本体
 */
ER
eBuiltInTest_builtInTest(void)
{
	ID		tskid, mtxid;
	ER		ercd;

	/*
	 *  タスク毎の検査
	 */
	for (tskid = TMIN_TSKID; tskid <= tmax_tskid; tskid++) {
		ercd = bit_mutex_task(tskid);
		if (ercd != E_OK) {
			return(ercd);
		}
	}

	/*
	 *  ミューテックス毎の検査
	 */
	for (mtxid = TMIN_MTXID; mtxid <= tmax_mtxid; mtxid++) {
		ercd = bit_mutex_mutex(mtxid);
		if (ercd != E_OK) {
			return(ercd);
		}
	}
	return(E_OK);
}
