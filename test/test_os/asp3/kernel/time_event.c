/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
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
 *  $Id: time_event.c 532 2016-01-15 14:48:04Z ertl-hiro $
 */

/*
 *		タイムイベント管理モジュール
 */

#include "kernel_impl.h"
#include "time_event.h"

/*
 *  TSTEP_HRTCNTの範囲チェック
 */
#if TSTEP_HRTCNT > 4000U
#error TSTEP_HRTCNT is too large.
#endif /* TSTEP_HRTCNT > 4000U */

/*
 *  HRTCNT_BOUNDの範囲チェック
 */
#if HRTCNT_BOUND >= 4294000000U
#error HRTCNT_BOUND is too large.
#endif /* HRTCNT_BOUND >= 4294000000U */

#ifdef TCYC_HRTCNT
#if HRTCNT_BOUND >= TCYC_HRTCNT
#error HRTCNT_BOUND is too large.
#endif /* HRTCNT_BOUND >= TCYC_HRTCNT */
#endif /* TCYC_HRTCNT */

/*
 *  タイムイベントヒープ操作マクロ
 */
#define PARENT(p_tmevtn)	(tmevt_heap + (((p_tmevtn) - tmevt_heap) >> 1))
													/* 親ノードを求める */
#define LCHILD(p_tmevtn)	(tmevt_heap + (((p_tmevtn) - tmevt_heap) << 1))
													/* 左の子ノードを求める */
/*
 *  タイムイベントヒープ中の先頭のノード
 */
#define p_top_tmevtn	(&(tmevt_heap[1]))
#define top_evttim		(tmevt_heap[1].p_tmevtb->evttim)
										/* 先頭のタイムイベントの発生時刻 */
/*
 *  タイムイベントヒープ中の最後のノード
 */
#define p_last_tmevtn	(tmevt_heap[0].p_last)

/*
 *  イベント時刻の前後関係の判定［ASPD1009］
 *
 *  イベント時刻は，boundary_evttimからの相対値で比較する．すなわち，
 *  boundary_evttimを最も早い時刻，boundary_evttim−1が最も遅い時刻とみ
 *  なして比較する．
 */
#define EVTTIM_ADVANCE(t)	((t) - boundary_evttim)
#define EVTTIM_LT(t1, t2)	(EVTTIM_ADVANCE(t1) < EVTTIM_ADVANCE(t2))
#define EVTTIM_LE(t1, t2)	(EVTTIM_ADVANCE(t1) <= EVTTIM_ADVANCE(t2))

#ifdef TOPPERS_tmeini

/*
 *  境界イベント時刻［ASPD1008］
 */
EVTTIM	boundary_evttim;

/*
 *  最後に現在時刻を算出した時点でのイベント時刻［ASPD1012］
 */
EVTTIM	current_evttim;

/*
 *  最後に現在時刻を算出した時点での高分解能タイマのカウント値［ASPD1012］
 */
HRTCNT	current_hrtcnt;

/*
 *  最も進んでいた時のイベント時刻［ASPD1041］
 */
EVTTIM	monotonic_evttim;

/*
 *  システム時刻のオフセット［ASPD1043］
 */
SYSTIM	systim_offset;

/*
 *  高分解能タイマ割込みの処理中であることを示すフラグ［ASPD1032］
 */
bool_t	in_signal_time;

/*
 *  タイムイベント管理モジュールの初期化［ASPD1061］
 */
void
initialize_tmevt(void)
{
	current_evttim = 0U;							/*［ASPD1047］*/
	boundary_evttim = current_evttim - BOUNDARY_MARGIN;
													/*［ASPD1048］*/
	monotonic_evttim = 0U;							/*［ASPD1046］*/
	systim_offset = 0U;								/*［ASPD1044］*/
	in_signal_time = false;							/*［ASPD1033］*/
	p_last_tmevtn = tmevt_heap;
}

#endif /* TOPPERS_tmeini */

/*
 *  タイムイベントの挿入位置を上向きに探索
 *
 *  時刻evttimに発生するタイムイベントを挿入するノードを空けるために，
 *  ヒープの上に向かって空ノードを移動させる．移動前の空ノードの位置を
 *  p_tmevtnに渡すと，移動後の空ノードの位置（すなわち挿入位置）を返す．
 */
#ifdef TOPPERS_tmeup

TMEVTN *
tmevt_up(TMEVTN *p_tmevtn, EVTTIM evttim)
{
	TMEVTN	*p_parent;

	while (p_tmevtn > p_top_tmevtn) {
		/*
		 *  親ノードのイベント発生時刻の方が早い（または同じ）ならば，
		 *  p_tmevtnが挿入位置なのでループを抜ける．
		 */
		p_parent = PARENT(p_tmevtn);
		if (EVTTIM_LE(p_parent->p_tmevtb->evttim, evttim)) {
			break;
		}

		/*
		 *  親ノードをp_tmevtnの位置に移動させる．
		 */
		*p_tmevtn = *p_parent;
		p_tmevtn->p_tmevtb->p_tmevtn = p_tmevtn;

		/*
		 *  p_tmevtnを親ノードの位置に更新．
		 */
		p_tmevtn = p_parent;
	}
	return(p_tmevtn);
}

#endif /* TOPPERS_tmeup */

/*
 *  タイムイベントの挿入位置を下向きに探索
 *
 *  時刻evttimに発生するタイムイベントを挿入するノードを空けるために，
 *  ヒープの下に向かって空ノードを移動させる．移動前の空ノードの位置を
 *  p_tmevtnに渡すと，移動後の空ノードの位置（すなわち挿入位置）を返す．
 */
#ifdef TOPPERS_tmedown

TMEVTN *
tmevt_down(TMEVTN *p_tmevtn, EVTTIM evttim)
{
	TMEVTN	*p_child;

	while ((p_child = LCHILD(p_tmevtn)) <= p_last_tmevtn) {
		/*
		 *  左右の子ノードのイベント発生時刻を比較し，早い方の子ノード
		 *  の位置をp_childに設定する．以下の子ノードは，ここで選ばれた
		 *  方の子ノードのこと．
		 */
		if (p_child + 1 <= p_last_tmevtn
					&& EVTTIM_LT((p_child + 1)->p_tmevtb->evttim,
											p_child->p_tmevtb->evttim)) {
			p_child = p_child + 1;
		}

		/*
		 *  子ノードのイベント発生時刻の方が遅い（または同じ）ならば，
		 *  p_tmevtnが挿入位置なのでループを抜ける．
		 */
		if (EVTTIM_LE(evttim, p_child->p_tmevtb->evttim)) {
			break;
		}

		/*
		 *  子ノードをp_tmevtnの位置に移動させる．
		 */
		*p_tmevtn = *p_child;
		p_tmevtn->p_tmevtb->p_tmevtn = p_tmevtn;

		/*
		 *  p_tmevtnを子ノードの位置に更新．
		 */
		p_tmevtn = p_child;
	}
	return(p_tmevtn);
}

#endif /* TOPPERS_tmedown */

/*
 *  タイムイベントヒープへの追加
 *
 *  p_tmevtbで指定したタイムイベントブロックを，タイムイベントヒープに
 *  追加する．
 */
Inline void
tmevtb_insert(TMEVTB *p_tmevtb)
{
	TMEVTN	*p_tmevtn;

	/*
	 *  p_last_tmevtnをインクリメントし，そこから上に挿入位置を探す．
	 */
	p_tmevtn = tmevt_up(++p_last_tmevtn, p_tmevtb->evttim);

	/*
	 *  タイムイベントをp_tmevtnの位置に挿入する．
	 */ 
	p_tmevtn->p_tmevtb = p_tmevtb;
	p_tmevtb->p_tmevtn = p_tmevtn;
}

/*
 *  タイムイベントヒープからの削除
 */
Inline void
tmevtb_delete(TMEVTB *p_tmevtb)
{
	TMEVTN	*p_tmevtn = p_tmevtb->p_tmevtn;
	TMEVTN	*p_parent;
	EVTTIM	event_evttim;

	/*
	 *  削除によりタイムイベントヒープが空になる場合は何もしない．
	 */
	if (--p_last_tmevtn < p_top_tmevtn) {
		return;
	}

	/*
	 *  削除したノードの位置に最後のノード（p_last_tmevtn + 1 の位置の
	 *  ノード）を挿入し，それを適切な位置へ移動させる．実際には，最後
	 *  のノードを実際に挿入するのではなく，削除したノードの位置が空ノー
	 *  ドになるので，最後のノードを挿入すべき位置へ向けて空ノードを移
	 *  動させる．
	 *
	 *  最後のノードのイベント発生時刻が，削除したノードの親ノードのイ
	 *  ベント発生時刻より前の場合には，上に向かって挿入位置を探す．そ
	 *  うでない場合には，下に向かって探す．
	 */
	event_evttim = (p_last_tmevtn + 1)->p_tmevtb->evttim;
	if (p_tmevtn > p_top_tmevtn
			&& EVTTIM_LT(event_evttim,
						(p_parent = PARENT(p_tmevtn))->p_tmevtb->evttim)) {
		/*
		 *  親ノードをp_tmevtnの位置に移動させる．
		 */
		*p_tmevtn = *p_parent;
		p_tmevtn->p_tmevtb->p_tmevtn = p_tmevtn;

		/*
		 *  削除したノードの親ノードから上に向かって挿入位置を探す．
		 */
		p_tmevtn = tmevt_up(p_parent, event_evttim);
	}
	else {
		/*
		 *  削除したノードから下に向かって挿入位置を探す．
		 */
		p_tmevtn = tmevt_down(p_tmevtn, event_evttim);
	}

	/*
	 *  最後のノードをp_tmevtnの位置に挿入する．
	 */ 
	*p_tmevtn = *(p_last_tmevtn + 1);
	p_tmevtn->p_tmevtb->p_tmevtn = p_tmevtn;
}

/*
 *  タイムイベントヒープの先頭のノードの削除
 */
Inline TMEVTB *
tmevtb_delete_top(void)
{
	TMEVTN	*p_tmevtn;
	TMEVTB	*p_top_tmevtb = p_top_tmevtn->p_tmevtb;
	EVTTIM	event_evttim;

	/*
	 *  削除によりタイムイベントヒープが空になる場合は何もしない．
	 */
	if (--p_last_tmevtn >= p_top_tmevtn) {
		/*
		 *  ルートノードに最後のノード（p_last_tmevtn + 1 の位置のノー
		 *  ド）を挿入し，それを適切な位置へ移動させる．実際には，最後
		 *  のノードを実際に挿入するのではなく，ルートノードが空ノード
		 *  になるので，最後のノードを挿入すべき位置へ向けて空ノードを
		 *  移動させる．
		 */
		event_evttim = (p_last_tmevtn + 1)->p_tmevtb->evttim;
		p_tmevtn = tmevt_down(p_top_tmevtn, event_evttim);

		/*
		 *  最後のノードをp_tmevtnの位置に挿入する．
		 */ 
		*p_tmevtn = *(p_last_tmevtn + 1);
		p_tmevtn->p_tmevtb->p_tmevtn = p_tmevtn;
	}
	return(p_top_tmevtb);
}

/*
 *  現在のイベント時刻の更新
 */
#ifdef TOPPERS_tmecur

void
update_current_evttim(void)
{
	HRTCNT	new_hrtcnt, hrtcnt_advance;
	EVTTIM	previous_evttim;

	new_hrtcnt = target_hrt_get_current();			/*［ASPD1013］*/
	hrtcnt_advance = new_hrtcnt - current_hrtcnt;	/*［ASPD1014］*/
#ifdef TCYC_HRTCNT
	if (new_hrtcnt < current_hrtcnt) {
		hrtcnt_advance += TCYC_HRTCNT;
	}
#endif /* TCYC_HRTCNT */

	previous_evttim = current_evttim;
	current_evttim += (EVTTIM) hrtcnt_advance;		/*［ASPD1015］*/
	current_hrtcnt = new_hrtcnt;					/*［ASPD1016］*/
	boundary_evttim = current_evttim - BOUNDARY_MARGIN;	/*［ASPD1011］*/

	if (monotonic_evttim - previous_evttim < (EVTTIM) hrtcnt_advance) {
#ifdef UINT64_MAX
		if (current_evttim < monotonic_evttim) {
			systim_offset += 1LLU << 32;			/*［ASPD1045］*/
		}
#endif /* UINT64_MAX */
		monotonic_evttim = current_evttim;			/*［ASPD1042］*/
	}
}

#endif /* TOPPERS_tmecur */

/*
 *  現在のイベント時刻を遅い方に丸めたイベント時刻の算出［ASPD1027］
 *
 *  現在のイベント時刻を更新した後に呼ぶことを想定している．
 */
Inline EVTTIM
calc_current_evttim_ub(void)
{
	return(current_evttim + ((EVTTIM) TSTEP_HRTCNT));
}

/*
 *  高分解能タイマ割込みの発生タイミングの設定
 */
#ifdef TOPPERS_tmeset

void
set_hrt_event(void)
{
	HRTCNT	hrtcnt;

	if (p_last_tmevtn < p_top_tmevtn) {
		target_hrt_set_event(HRTCNT_BOUND);			/*［ASPD1007］*/
	}
	else if (EVTTIM_LE(top_evttim, current_evttim)) {
		target_hrt_raise_event();					/*［ASPD1017］*/
	}
	else {
		hrtcnt = (HRTCNT)(top_evttim - current_evttim);
		if (hrtcnt > HRTCNT_BOUND) {
			target_hrt_set_event(HRTCNT_BOUND);		/*［ASPD1006］*/
		}
		else {
			target_hrt_set_event(hrtcnt);			/*［ASPD1002］*/
		}
	}
}

#endif /* TOPPERS_tmeset */

/*
 *  タイムイベントブロックのヒープへの挿入
 */
#ifdef TOPPERS_tmereg

void
tmevtb_register(TMEVTB *p_tmevtb)
{
	tmevtb_insert(p_tmevtb);
}

#endif /* TOPPERS_tmereg */

/*
 *  相対時間指定によるタイムイベントの登録
 *  
 */
#ifdef TOPPERS_tmeenq

void
tmevtb_enqueue(TMEVTB *p_tmevtb, RELTIM time)
{
	/*
	 *  現在のイベント時刻とタイムイベントの発生時刻を求める［ASPD1026］．
	 */
	update_current_evttim();
	p_tmevtb->evttim = calc_current_evttim_ub() + time;

	/*
	 *  タイムイベントブロックをヒープに挿入する［ASPD1030］．
	 */
	tmevtb_insert(p_tmevtb);

	/*
	 *  高分解能タイマ割込みの発生タイミングを設定する［ASPD1031］
	 *  ［ASPD1034］．
	 */
	if (!in_signal_time && p_tmevtb->p_tmevtn == p_top_tmevtn) {
		set_hrt_event();
	}
}

#endif /* TOPPERS_tmeenq */

/*
 *  タイムイベントの登録解除
 */
#ifdef TOPPERS_tmedeq

void
tmevtb_dequeue(TMEVTB *p_tmevtb)
{
	TMEVTN	*p_tmevtn;

	/*
	 *  タイムイベントブロックをヒープから削除する［ASPD1039］．
	 */
	p_tmevtn = p_tmevtb->p_tmevtn;
	tmevtb_delete(p_tmevtb);

	/*
	 *  高分解能タイマ割込みの発生タイミングを設定する［ASPD1040］．
	 */
	if (!in_signal_time && p_tmevtn == p_top_tmevtn) {
		update_current_evttim();
		set_hrt_event();
	}
}

#endif /* TOPPERS_tmedeq */

/*
 *  システム時刻の調整時のエラーチェック
 */
#ifdef TOPPERS_tmechk

bool_t
check_adjtim(int_t adjtim)
{
	if (adjtim > 0) {
		return(p_last_tmevtn >= p_top_tmevtn	/*［NGKI3588］*/
					&& EVTTIM_LE(top_evttim, current_evttim - TMAX_ADJTIM));
	}
	else if (adjtim < 0) {						/*［NGKI3589］*/
		return(monotonic_evttim - current_evttim >= -TMIN_ADJTIM);
	}
	return(false);
}

#endif /* TOPPERS_tmechk */

/*
 *  タイムイベントが発生するまでの時間の計算
 */
#ifdef TOPPERS_tmeltim

RELTIM
tmevt_lefttim(TMEVTB *p_tmevtb)
{
	EVTTIM	evttim, current_evttim_ub;

	/*
	 *  現在のイベント時刻を遅い方に丸めた時刻を求める［ASPD1050］．
	 */
	update_current_evttim();
	current_evttim_ub = calc_current_evttim_ub();

	/*
	 *  タイムイベント発生までの相対時間を求める［ASPD1049］．
	 */
	evttim = p_tmevtb->evttim;
	if (EVTTIM_LE(evttim, current_evttim_ub)) {
		/*
		 *  タイムイベントの発生時刻を過ぎている場合には0を返す［NGKI0552］．
		 */
		return(0U);
	}
	else {
		return((RELTIM)(evttim - current_evttim_ub));
	}
}

#endif /* TOPPERS_tmeltim */

/*
 *  高分解能タイマ割込みの処理
 */
#ifdef TOPPERS_sigtim

void
signal_time(void)
{
	TMEVTB	*p_tmevtb;
	bool_t	callflag;

	assert(sense_context());
	assert(!sense_lock());

	lock_cpu();
	in_signal_time = true;							/*［ASPD1033］*/

	do {
		/*
		 *  コールバック関数を呼び出さなければループを抜ける［ASPD1020］．
		 */
		callflag = false;

		/*
		 *  現在のイベント時刻を求める［ASPD1022］．
		 */
		update_current_evttim();

		/*
		 *  発生時刻がcurrent_evttim以前のタイムイベントがあれば，タイ
		 *  ムイベントヒープから削除し，コールバック関数を呼び出す
		 *  ［ASPD1018］［ASPD1019］．
		 */
		while (p_last_tmevtn >= p_top_tmevtn
							&& EVTTIM_LE(top_evttim, current_evttim)) {
			p_tmevtb = tmevtb_delete_top();
			(*(p_tmevtb->callback))(p_tmevtb->arg);
			callflag = true;
		}
	} while (callflag);								/*［ASPD1020］*/

	/*
	 *  高分解能タイマ割込みの発生タイミングを設定する［ASPD1025］．
	 */
	set_hrt_event();

	in_signal_time = false;							/*［ASPD1033］*/
	unlock_cpu();
}

#endif /* TOPPERS_sigtim */
