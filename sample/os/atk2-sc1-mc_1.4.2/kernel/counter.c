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
 *  $Id: counter.c 778 2017-03-06 07:21:41Z nces-hibino $
 */

/*
 *		カウンタ制御モジュール
 */

#include "kernel_impl.h"
#include "task.h"
#include "counter.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_INCCNT_ENTER
#define LOG_INCCNT_ENTER(cntid)
#endif /* LOG_INCCNT_ENTER */

#ifndef LOG_INCCNT_LEAVE
#define LOG_INCCNT_LEAVE(ercd)
#endif /* LOG_INCCNT_LEAVE */

#ifndef LOG_GETCNT_ENTER
#define LOG_GETCNT_ENTER(cntid, p_val)
#endif /* LOG_GETCNT_ENTER */

#ifndef LOG_GETCNT_LEAVE
#define LOG_GETCNT_LEAVE(ercd, val)
#endif /* LOG_GETCNT_LEAVE */

#ifndef LOG_GETECT_ENTER
#define LOG_GETECT_ENTER(cntid, val, p_eval)
#endif /* LOG_GETECT_ENTER */

#ifndef LOG_GETECT_LEAVE
#define LOG_GETECT_LEAVE(ercd, val, eval)
#endif /* LOG_GETECT_LEAVE */

/*
 *  カウンタ満了キューへの挿入
 */
#ifdef TOPPERS_insert_cnt_expr_que

void
insert_cnt_expr_que(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb)
{
	TickType	enqval, curval;
	QUEUE		*next;
	CounterType cntid;

	enqval = p_cntexpinfo->expiretick;

	cntid = CNTID(p_cntcb);
	if (!is_hwcnt(cntid)) {
		curval = p_cntcb->curval;
	}
	else {
		curval = get_curval(p_cntcb, cntid);
	}

	/* 挿入場所のサーチ */
	next = p_cntcb->cntexpque.p_next;

	if (curval < enqval) {
		/* カウンタのオーバーフローが起こらない場合 */
		while ((next != &(p_cntcb->cntexpque)) &&
			   (diff_tick(curval, ((CNTEXPINFO *) next)->expiretick, p_cntcb->p_cntinib->maxval2) <= p_cntcb->p_cntinib->maxval ||
				((curval <= ((CNTEXPINFO *) next)->expiretick) &&
				 (((CNTEXPINFO *) next)->expiretick <= enqval)))) {
			next = next->p_next;
		}
	}
	else {
		/* カウンタのオーバーフローが起こる場合 */
		while ((next != &(p_cntcb->cntexpque)) &&
			   ((diff_tick(curval, ((CNTEXPINFO *) next)->expiretick, p_cntcb->p_cntinib->maxval2) <= p_cntcb->p_cntinib->maxval) ||
				(curval <= ((CNTEXPINFO *) next)->expiretick) ||
				(((CNTEXPINFO *) next)->expiretick <= enqval))) {
			next = next->p_next;
		}
	}

	queue_insert_prev(next, &(p_cntexpinfo->cntexpque));

	/*
	 *  ハードウェアカウンタかつ先頭に挿入した場合,再度ハードウェアカウンタに
	 *  満了時間を設定し直す必要がある
	 */
	if (is_hwcnt(cntid) && (p_cntcb->cntexpque.p_next == &(p_cntexpinfo->cntexpque))) {

		/* 現在設定されている時刻をキャンセル */
		(hwcntinib_table[cntid].cancel)();

		/* 先頭に挿入した時刻に再設定 */
		(hwcntinib_table[cntid].set)(enqval);
		p_cntcb->hwset = TRUE;

		/*
		 *  再設定中に次の満了時間を過ぎてしまったかチェック
		 *
		 *  過ぎてしまった場合, 強制割込みにより満了処理を実行する
		 *  またsetした時間とgetした時間が同じであった場合,
		 *  ハードウェアで取りこぼしていると想定し, 強制割込みを発生させる
		 *  ハードウェアで取りこぼしていない場合に強制割込みを起こしても問題ない
		 */
		if (diff_tick((hwcntinib_table[cntid].get)(), enqval,
					  p_cntcb->p_cntinib->maxval2) <= p_cntcb->p_cntinib->maxval) {
			/* 現在設定されている時刻をキャンセル */
			(hwcntinib_table[cntid].cancel)();
			/* 強制割込みを発生させる */
			(hwcntinib_table[cntid].trigger)();
		}
	}
}

#endif /* TOPPERS_insert_cnt_expr_que */

/*
 *  カウンタ満了キューから削除
 */
#ifdef TOPPERS_delete_cnt_expr_que

void
delete_cnt_expr_que(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb)
{
	CounterType cntid;
	QUEUE		*p_cntexpque;

	/* カウンタキューから満了処理を削除する前の先頭キュー保持 */
	p_cntexpque = p_cntcb->cntexpque.p_next;

	queue_delete(&(p_cntexpinfo->cntexpque));
	queue_initialize(&(p_cntexpinfo->cntexpque));

	/*
	 *  ハードウェアカウンタかつ削除する満了処理はカウンタ満了の
	 *  先頭の場合，タイマをキャンセルする
	 */
	cntid = CNTID(p_cntcb);
	if (is_hwcnt(cntid) && (p_cntcb->cntexpque.p_next != p_cntexpque)) {

		/* 現在設定されている時刻をキャンセル */
		(hwcntinib_table[cntid].cancel)();

		/* ペンディング中の割込み要求をキャンセル */
		(hwcntinib_table[cntid].intcancel)();

		/*
		 *  p_cntexpinfoで指定された満了処理削除後，カウンタ満了次の
		 *  満了処理の満了点を設定する
		 */
		if (queue_empty(&(p_cntcb->cntexpque)) == FALSE) {

			/* 先頭に挿入した時刻に再設定 */
			(hwcntinib_table[cntid].set)(((CNTEXPINFO *) p_cntcb->cntexpque.p_next)->expiretick);
			p_cntcb->hwset = TRUE;

			/*
			 *  再設定中に次の満了時間を過ぎてしまったかチェック
			 *
			 *  過ぎてしまった場合, 強制割込みにより満了処理を実行する
			 *  またsetした時間とgetした時間が同じであった場合,
			 *  ハードウェアで取りこぼしていると想定し, 強制割込みを発生させる
			 *  ハードウェアで取りこぼしていない場合に強制割込みを起こしても問題ない
			 */
			if (diff_tick((hwcntinib_table[cntid].get)(),
						  ((CNTEXPINFO *) p_cntcb->cntexpque.p_next)->expiretick,
						  p_cntcb->p_cntinib->maxval2) <= p_cntcb->p_cntinib->maxval) {
				/* 現在設定されている時刻をキャンセル */
				(hwcntinib_table[cntid].cancel)();
				/* 強制割込みを発生させる */
				(hwcntinib_table[cntid].trigger)();
			}
		}
	}

}

#endif /* TOPPERS_delete_cnt_expr_que */

/*
 *  カウンタ機能の初期化
 */
#ifdef TOPPERS_counter_initialize

void
counter_initialize(void)
{
	CounterType	i;
	CNTCB		*p_cntcb;
	CoreIdType	coreid = x_core_id();

	for (i = 0U; i < tnum_counter; i++) {
		if (cntinib_table[i].coreid == coreid) {
			p_cntcb = p_cntcb_table[i];
			p_cntcb->p_cntinib = &(cntinib_table[i]);
			p_cntcb->curval = 0U;
			queue_initialize(&(p_cntcb->cntexpque));
			p_cntcb->cstat = CS_NULL;
		}
	}

	for (i = 0U; i < tnum_hardcounter; i++) {
		if (hwcntinib_table[i].coreid == coreid) {
			(hwcntinib_table[i].init)(cntinib_table[i].maxval2,
									  hwcntinib_table[i].nspertick);
			(hwcntinib_table[i].start)();
		}
	}

}

#endif /* TOPPERS_counter_initialize */

/*
 *  カウンタ機能の終了処理
 */
#ifdef TOPPERS_counter_terminate

void
counter_terminate(void)
{
	CounterType i;

	for (i = 0U; i < tnum_hardcounter; i++) {
		(hwcntinib_table[i].stop)();
	}
}

#endif /* TOPPERS_counter_terminate */

/*
 *  指定した相対時間からのカウンタ値取得(APIからの取得)
 *
 *  指定したカウンタの現在値と, 渡された相対値を足しこみ更新値を
 *  戻り値として返す
 */
#ifdef TOPPERS_get_reltick

TickType
get_reltick(const CNTCB *p_cntcb, TickType relval)
{
	CounterType	cntid;
	TickType	result;
	TickType	curval;

	cntid = CNTID(p_cntcb);

	curval = get_curval(p_cntcb, cntid);

	/* 現在時間から指定されたオフセット分過ぎた時間を算出する */
	result = add_tick(curval, relval, p_cntcb->p_cntinib->maxval2);

	return(result);
}

#endif /* TOPPERS_get_reltick */

/*
 *  指定した絶対時間からのカウンタ値取得(APIからの取得)
 *
 *  引数で渡された絶対値を指定したカウンタの現在値に変換し
 *  更新値を戻り値として返す
 */
#ifdef TOPPERS_get_abstick

TickType
get_abstick(const CNTCB *p_cntcb, TickType absval)
{
	CounterType	cntid;
	TickType	result;
	TickType	curval;
	TickType	nextval;

	cntid = CNTID(p_cntcb);

	curval = get_curval(p_cntcb, cntid);

	/* maxval2を考慮した絶対時間に変換 */
	nextval = absval +  p_cntcb->p_cntinib->maxval + 1U;

	if (curval < (p_cntcb->p_cntinib->maxval + 1U)) {
		/*
		 *  カウンタの現在値が0〜maxvalの間の場合，
		 *  絶対時刻に未到達なので，絶対時刻を返す
		 */
		if (absval > curval) {
			result = absval;
		}
		else {
			result = nextval;
		}
	}
	else {
		/*
		 *  カウンタの現在値がmaxval〜maxval2の間の場合，
		 *  maxval2考慮した絶対も超えたので，絶対時刻を返す
		 */
		if (nextval <= curval) {
			result = absval;
		}
		else {
			result = nextval;
		}
	}

	return(result);
}

#endif /* TOPPERS_get_abstick */

/*
 *  カウンタの満了処理
 */
#ifdef TOPPERS_expire_process

void
expire_process(CNTCB *p_cntcb, CounterType cntid)
{
	CNTEXPINFO	*p_cntexpinfo;
	TickType	nowval;
	boolean		dspreq;
	CCB			*p_ccb = get_p_ccb(p_cntcb->p_cntinib->coreid);

	p_cntcb->hwset = FALSE;
	nowval = get_curval(p_cntcb, cntid);

	/*
	 *  カウンタの満了処理
	 *
	 *  キューが空でなく, リアルタイムな現在時間から見てキューの先頭の満了
	 *  時間が既に過ぎていれば, 満了処理を実行する
	 *
	 *  リアルタイムな現在時間をその都度取得するため,キューの先頭満了処理
	 *  の満了時間を再設定する時に目的の満了時間を超えてしまってもカバー
	 *  できる
	 */
	while ((queue_empty(&(p_cntcb->cntexpque)) == FALSE) &&
		   (diff_tick(nowval, ((CNTEXPINFO *) p_cntcb->cntexpque.p_next)->expiretick,
					  p_cntcb->p_cntinib->maxval2) <= p_cntcb->p_cntinib->maxval)) {

		/* カウンタ満了キューの先頭の満了処理を，キューから外す */
		p_cntexpinfo = (CNTEXPINFO *) p_cntcb->cntexpque.p_next;
		queue_delete(&(p_cntexpinfo->cntexpque));
		queue_initialize(&(p_cntexpinfo->cntexpque));

		if (is_hwcnt(cntid)) {

			/*
			 *  次の満了点の設定
			 *  次の満了点が実時間を経過していない場合設定する
			 */
			if ((queue_empty(&(p_cntcb->cntexpque)) == FALSE) && ((diff_tick((hwcntinib_table[cntid].get)(),
																			 ((CNTEXPINFO *) p_cntcb->cntexpque.p_next)->expiretick, p_cntcb->p_cntinib->maxval2)) >
																  p_cntcb->p_cntinib->maxval)) {
				(hwcntinib_table[cntid].set)(((CNTEXPINFO *) p_cntcb->cntexpque.p_next)->expiretick);
				p_cntcb->hwset = TRUE;
			}
		}

		/* カウンタ満了処理呼出し */
		(p_cntexpinfo->expirefunc)(p_cntexpinfo, p_cntcb);

		/*
		 *  タスクからの呼び出し時，高優先度タスクレディ状態になった場合あるので，
		 *  チェックしてディスパッチする
		 */
		release_cnt_lock(p_ccb);
		acquire_tsk_lock(p_ccb);
		if ((p_ccb->p_runtsk != p_ccb->p_schedtsk) && (p_ccb->callevel_stat == TCL_TASK)) {
			dspreq = TRUE;
		}
		else {
			dspreq = FALSE;
		}
		release_tsk_lock_and_dispatch(p_ccb, dspreq);

		/*
		 *  割込みレスポンス考慮し，1個の満了点処理後に
		 *  1回の割込許可/禁止を実施
		 */
		x_nested_unlock_os_int();
		x_nested_lock_os_int();
		acquire_cnt_lock(p_ccb);

		nowval = get_curval(p_cntcb, cntid);
	}

	if (is_hwcnt(cntid) && (queue_empty(&(p_cntcb->cntexpque)) == FALSE) && (p_cntcb->hwset == FALSE)) {
		(hwcntinib_table[cntid].set)(((CNTEXPINFO *) p_cntcb->cntexpque.p_next)->expiretick);
	}
}
#endif /* TOPPERS_expire_process */
