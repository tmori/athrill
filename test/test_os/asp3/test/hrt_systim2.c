/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: hrt_systim2.c 310 2015-02-08 13:46:46Z ertl-hiro $
 */

/* 
 *		システム時刻管理機能のテスト(2)
 *
 * 【テストの目的】
 *
 *	システム時刻管理機能の標準のサービスコール（set_tim，get_tim，
 *	adj_tim）を，要求ベースでテストする．
 *
 *	テスト対象のサービスコール処理関数およびそこから呼び出される以下の
 *	関数のC1カバレッジを達成する．
 *		set_tim … 関数内に分岐がない
 *		get_tim … 関数内に分岐がない
 *		adj_tim
 *		check_adjtim
 *		update_current_evttim（TCYC_HRTCNTが定義されていない場合）
 *
 * 【テスト項目】
 *
 *  (A) set_timの要求ベーステスト
 *	  (A-1) 非タスクコンテキストからの呼出し［NGKI3564］
 *	  (A-2) CPUロック状態からの呼出し［NGKI3565］
 *	  (A-3) システム時刻の現在値が正しく設定されること［NGKI3567］
 *  (B) get_timの要求ベーステスト
 *	  (B-1) 非タスクコンテキストからの呼出し［NGKI2350］
 *	  (B-2) CPUロック状態からの呼出し［NGKI2351］
 *	  (B-3) システム時刻の現在値が正しく参照されること［NGKI2354］
 *	  (B-4) adj_timによってシステム時刻を戻した場合，システム時刻が最も
 *			進んでいた時のシステム時刻を返すこと［NGKI3591］
 *	  (B-5) 参照するシステム時刻の進みが止まっている間に，set_timによ
 *			りシステム時刻を設定した場合でも，参照するシステム時刻の進
 *			みが止まっている時間は変化しないこと［NGKI3592］
 *  (C) adj_timの要求ベーステスト
 *	  (C-1) CPUロック状態からの呼出し［NGKI3583］
 *	  (C-2) adjtimが小さすぎる［NGKI3584］
 *	  (C-3) adjtimが大きすぎる［NGKI3584］
 *	  (C-4) システム時刻にadjtimが加えられること［NGKI3586］
 *	  (C-5) システム時刻の経過をきっかけに発生するタイムイベントが発生
 *			するまでの相対時間も調整されること［NGKI3587］
 *	  (C-6) (adj_tim > 0)で，1秒以上過去の発生時刻を持つタイムイベント
 *			が残っている場合に，E_OBJエラーとなること［NGKI3588］
 *	  (C-7) (adj_tim < 0)で，現在のシステム時刻が，システム時刻が最も進
 *			んでいた時のシステム時刻より1秒以上戻っている場合に，E_OBJ
 *			エラーとなること［NGKI3589］
 *	  (C-8) 非タスクコンテキストからの呼出しで，システム時刻にadjtimが
 *			加えられること
 *  (D) adj_timの実行/分岐パスの網羅
 *	  (D-1) (adjtim <= 0)の場合
 *	  (D-2) (adjtim > 0)で，最も進んでいた時のイベント時刻の更新が必要
 *			ない場合
 *	  (D-3) (adjtim > 0)で，最も進んでいた時のイベント時刻の更新が必要
 *			で，システム時刻のオフセットを進める必要がない場合
 *	  (D-4) (adjtim > 0)で，最も進んでいた時のイベント時刻の更新が必要
 *			で，システム時刻のオフセットを進める必要がある場合
 *  (E) check_adjtimの実行/分岐パス/戻り値の網羅
 *	  (E-1) (adjtim > 0)で，タイムイベントが登録されていない場合
 *	  (E-2) (adjtim > 0)で，先頭のタイムイベントの発生時刻が1秒前以降で
 *			ある場合
 *	  (E-3) (adjtim > 0)で，先頭のタイムイベントの発生時刻が1秒前以前で
 *			ある場合（adj_timがE_OBJエラーとなる）
 *	  (E-4) (adjtim < 0)で，現在のシステム時刻が，システム時刻が最も進
 *			んでいた時のシステム時刻より1秒以上戻っていない場合
 *	  (E-5) (adjtim < 0)で，現在のシステム時刻が，システム時刻が最も進
 *			んでいた時のシステム時刻より1秒以上戻っている場合（adj_tim
 *			がE_OBJエラーとなる）
 *	  (E-6) (adjtim == 0)の場合
 *  (F) update_current_evttimの実行/分岐パスの網羅
 *	  (F-1) 最も進んでいた時のイベント時刻の更新が必要ない場合
 *	  (F-2) 最も進んでいた時のイベント時刻の更新が必要で，システム時刻
 *			のオフセットを進める必要がない場合
 *	  (F-3) 最も進んでいた時のイベント時刻の更新が必要で，システム時刻
 *			のオフセットを進める必要がある場合
 *
 * 【使用リソース】
 *
 *	高分解能タイマモジュールの性質：HRT_CONFIG1
 *		TCYC_HRTCNT		未定義（2^32の意味）
 *		TSTEP_HRTCNT	1U
 *		HRTCNT_BOUND	4000000002U
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	ALM1:  アラームハンドラ
 *	ALM2:  アラームハンドラ
 *	ALM3:  アラームハンドラ
 *
 * 【補足説明】
 *
 *	タイムイベントが登録されていない時に高分解能タイマに設定する相対は，
 *	ドリフト調整機能を持たない場合はHRTCNT_BOUNDであるのに対して，ドリ
 *	フト調整機能を持つ場合はTMAX_RELTIMをイベント時刻の伸縮率で割った値
 *	とHRTCNT_BOUNDの小さい方の値（このテストでは，ドリフト量を設定せず，
 *	HRTCNT_BOUND＞TMAX_RELTIMであるため，TMAX_RELTIMに一致）となる．そ
 *	こで，HRTCNT_EMPTYをこの値に定義し，ドリフト調整機能の有無によらず
 *	同じテストシーケンスが使えるようにする．
 *
 *	以下のテストシーケンスのコメント中で，「発生：xxx」とは，高分解能タ
 *	イマのカウント値がxxxになった時にタイムイベントが発生することを意味
 *	する．タイムイベントのイベント発生時刻ではない．
 *
 * 【テストシーケンス】
 *
 *	== START ==
 *	// カーネル起動．高分解能タイマのカウント値とイベント時刻は10ずれる
 *	1:		[target_hrt_get_current -> 10U]
 *	2:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（優先度：中）==
 *	// CPUロック状態で各サービスコールを呼び出す
 *	3:	loc_cpu()
 *		set_tim(1000000LLU) -> E_CTX				... (A-2)
 *		get_tim(&systim) -> E_CTX					... (B-2)
 *		adj_tim(100) -> E_CTX						... (C-1)
 *		unl_cpu()
 *	// まずはget_timの一般的な動作を確認
 *	4:	get_tim(&systim)							... (B-3)
 *	5:		[target_hrt_get_current -> 40U]
 *	6:	assert(systim == 40U - 10U)
 *	// システム時刻を32ビットを超える値に設定
 *		set_tim(2LLU << 32)							... (A-3)
 *	7:		[target_hrt_get_current -> 50U]			... (F-2)
 *	// システム時刻が期待通りに設定されていることを確認
 *	8:	get_tim(&systim)							... (B-3)
 *	9:		[target_hrt_get_current -> 60U]
 *	10:	assert(systim == (2LLU << 32) + 10U)
 *	11:	DO(target_raise_hrt_int(1U))
 *		tslp_tsk(TMAX_RELTIM) -> E_TMOUT
 *	12:		[target_hrt_get_current -> 70U]			// TMOUTの発生：4000000071U
 *	13:		[target_hrt_set_event <- 4000000001U]
 *	// ここで長時間経過したことを想定
 *	== HRT_HANDLER ==
 *	14:		[target_hrt_get_current -> 4000000080U]	// TMOUTが発生
 *	// ここでタイムアウト処理が行われる
 *	15:		[target_hrt_get_current -> 4000000090U]
 *	16:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	17:	get_tim(&systim)							... (B-3)
 *	18:		[target_hrt_get_current -> 4000000100U]
 *	19:	assert(systim == (2LLU << 32) + 4000000050U)
 *	// ここで時間が経過して，HRTが巡回したことを想定
 *	20:	get_tim(&systim)							... (B-3)
 *	21:		[target_hrt_get_current -> 100U]		... (F-3)
 *	22:	assert(systim == (3LLU << 32) + 50U)
 *	// adj_timのパラメータエラーのテスト
 *		adj_tim(-1000001) -> E_PAR					... (C-2)
 *		adj_tim(+1000001) -> E_PAR					... (C-3)
 *
 *	// adj_timでシステム時刻を進めるテスト
 *	// タイムイベントを1つ登録
 *		sta_alm(ALM1, 2000000U)
 *	23:		[target_hrt_get_current -> 120U]		// ALM1の発生：2,000,121
 *	24:		[target_hrt_set_event <- 2000001U]
 *	25:	get_tim(&systim)
 *	26:		[target_hrt_get_current -> 130U]
 *	27:	assert(systim == (3LLU << 32) + 80U)
 *	// adj_timでシステム時刻を進める
 *		adj_tim(+1000000)							... (C-4)(D-3)(E-2)
 *	28:		[target_hrt_get_current -> 140U]		// ALM1の発生：1,000,121
 *	29:		[target_hrt_set_event <- 999981U]		... (C-5)
 *	30:	get_tim(&systim)
 *	31:		[target_hrt_get_current -> 150U]
 *	32:	assert(systim == (3LLU << 32) + 1000000U + 100U)
 *	// タイムイベント発生までの時間をチェック
 *		ref_alm(ALM1, &ralm)
 *	33:		[target_hrt_get_current -> 160U]
 *	34:	assert(ralm.lefttim == 999960U)
 *	// ここで長時間経過したことを想定
 *		DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	35:		[target_hrt_get_current -> 1000130U]	// ALM1が発生
 *	== ALM1-1（1回目）==
 *	// 非タスクコンテキストから各サービスコールを呼び出す
 *	36:	set_tim(1LLU << 32) -> E_CTX				... (A-1)
 *		get_tim(&systim) -> E_CTX					... (B-1)
 *		RETURN
 *	37:		[target_hrt_get_current -> 1000140U]
 *	38:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *
 *	// adj_timでシステム時刻を戻すテスト
 *	// タイムイベントを1つ登録
 *	39:	sta_alm(ALM1, 2000000U)
 *	40:		[target_hrt_get_current -> 1000150U]	// ALM1の発生：3,000,151
 *	41:		[target_hrt_set_event <- 2000001U]
 *	// システム時刻を確認しておく
 *	42:	get_tim(&systim)
 *	43:		[target_hrt_get_current -> 1000160U]
 *	44:	assert(systim == (3LLU << 32) + 1000000U + 1000110U)
 *	// adj_timでシステム時刻を戻す
 *		adj_tim(-1000000)							... (C-4)(D-1)(E-4)
 *	45:		[target_hrt_get_current -> 1000170U]	// ALM1の発生：4,000,151
 *	46:		[target_hrt_set_event <- 2999981U]		... (C-5)
 *	// システム時刻が戻っていないことをチェック
 *	// adj_timを呼び出した時点のシステム時刻になっているはず
 *	47:	get_tim(&systim)							... (B-4)
 *	48:		[target_hrt_get_current -> 1000180U]
 *	49:	assert(systim == (3LLU << 32) + 1000000U + 1000120U)
 *	// タイムイベント発生までの時間をチェック
 *		ref_alm(ALM1, &ralm)
 *	50:		[target_hrt_get_current -> 1000190U]
 *		assert(ralm.lefttim == 2999960U)
 *	// ここで時間が経過したことを想定
 *	// システム時刻が進んでいないことをチェック
 *		get_tim(&systim)							... (B-4)
 *	51:		[target_hrt_get_current -> 1500200U]	... (F-1)
 *	52:	assert(systim == (3LLU << 32) + 1000000U + 1000120U)
 *	// ここで時間が経過したことを想定
 *	// システム時刻が進んでいることをチェック
 *		get_tim(&systim)							... (B-4)
 *	53:		[target_hrt_get_current -> 2000210U]
 *	54:	assert(systim == (3LLU << 32) + 2000160U)
 *	// adj_timでシステム時刻を戻す
 *		adj_tim(-1000000)							... (C-4)(D-1)(E-4)
 *	55:		[target_hrt_get_current -> 2000220U]	// ALM1の発生：5,000,151
 *	56:		[target_hrt_set_event <- 2999931U]		... (C-5)
 *	// システム時刻が戻っていないことをチェック
 *	// adj_timを呼び出した時点のシステム時刻になっているはず
 *		get_tim(&systim)							... (B-4)
 *	57:		[target_hrt_get_current -> 2000230U]
 *	58:	assert(systim == (3LLU << 32) + 2000170U)
 *	// ここでシステム時刻を設定
 *	// この時点で，get_timで参照するシステム時刻は20止まっていた
 *	// これ以降，さらに999980の間，システム時刻は止まっているはず
 *		set_tim(4LLU << 32)
 *	59:		[target_hrt_get_current -> 2000240U]
 *	// システム時刻が進んでいないことをチェック
 *		get_tim(&systim)							... (B-5)
 *	60:		[target_hrt_get_current -> 2000250U]
 *	61:	assert(systim == (4LLU << 32))
 *	// ここで時間が経過したことを想定
 *	// システム時刻が進んでいないことをチェック
 *		get_tim(&systim)							... (B-5)
 *	62:		[target_hrt_get_current -> 2500250U]
 *	63:	assert(systim == (4LLU << 32))
 *	// ここで時間が経過したことを想定
 *	// システム時刻が進んでいることをチェック
 *		get_tim(&systim)							... (B-5)
 *	64:		[target_hrt_get_current -> 3000260U]
 *	65:	assert(systim == (4LLU << 32) - 999980U + 1000020U)
 *
 *	// adj_timで繰り返しシステム時刻を進めるテスト
 *	// タイムイベントを2つ登録
 *		sta_alm(ALM1, 1000U)
 *	66:		[target_hrt_get_current -> 3000270U]	// ALM1の停止
 *	67:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	68:		[target_hrt_get_current -> 3000270U]	// ALM1の発生：3,001,271
 *	69:		[target_hrt_set_event <- 1001U]
 *	70:	sta_alm(ALM2, 1000U)
 *	71:		[target_hrt_get_current -> 3000280U]	// ALM2の発生：3,001,281
 *	// ここで時間が経過したことを想定
 *	72:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	73:		[target_hrt_get_current -> 3001280U]	// ALM1が発生
 *	== ALM1-2（2回目）==
 *	74:	adj_tim(+1000000)							... (C-8)(D-3)(E-2)
 *	75:		[target_hrt_get_current -> 3001290U]	// ALM2の発生：2,001,281
 *	76:	adj_tim(+1000000) -> E_OBJ					... (C-6)(E-3)
 *	77:		[target_hrt_get_current -> 3001300U]
 *	78:	RETURN
 *	== ALM2-1（1回目）==
 *	79:	RETURN
 *	80:		[target_hrt_get_current -> 3001400U]
 *	81:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *
 *	// (D-4)のテスト
 *	// ここで長時間経過したことを想定
 *	82:	DO(target_raise_hrt_int(0U))
 *	== HRT_HANDLER ==
 *	83:		[target_hrt_get_current -> -100U]
 *	84:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	== TASK1（続き）==
 *	// システム時刻を設定する（この後の計算を楽にするため）
 *	85:	set_tim(5LLU << 32)
 *	86:		[target_hrt_get_current -> -90U]
 *	// adj_timでシステム時刻を進める
 *	87:	adj_tim(200)								... (C-4)(D-4)(E-1)
 *	88:		[target_hrt_get_current -> -80U]
 *	89:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	// システム時刻を確認
 *	90:	get_tim(&systim)
 *	91:		[target_hrt_get_current -> -70U]
 *	92:	assert(systim == (5LLU << 32) + 200U + 20U)
 *
 *	// adj_timで繰り返しシステム時刻を戻すテスト
 *	// ここで少し時間が経過したことを想定
 *	// システム時刻を設定する（この後の計算を楽にするため）
 *	93:	set_tim(6LLU << 32)
 *	94:		[target_hrt_get_current -> 110U]
 *	// adj_timでシステム時刻を戻す
 *	95:	adj_tim(-200)								... (C-4)(D-1)(E-4)
 *	96:		[target_hrt_get_current -> 120U]
 *	97:		[target_hrt_set_event <- HRTCNT_EMPTY]
 *	// システム時刻を確認
 *	98:	get_tim(&systim)
 *	99:		[target_hrt_get_current -> 130U]
 *	100:assert(systim == (6LLU << 32) + 10U)
 *	// adj_timでシステム時刻をさらに戻す
 *		adj_tim(-1000000)							... (C-4)(D-1)(E-4)
 *	101:	[target_hrt_get_current -> 140U]
 *	102:	[target_hrt_set_event <- HRTCNT_EMPTY]
 *	103:adj_tim(-1000000) -> E_OBJ					... (C-7)(E-5)
 *	104:	[target_hrt_get_current -> 150U]
 *	105:adj_tim(+1000000)							... (C-4)(D-2)(E-1)
 *	106:	[target_hrt_get_current -> 160U]
 *	107:	[target_hrt_set_event <- HRTCNT_EMPTY]
 *	// システム時刻を確認
 *	// adj_tim(-200)を呼び出した時点でのシステム時刻になっているはず
 *	108:get_tim(&systim)
 *	109:	[target_hrt_get_current -> 170U]
 *	110:assert(systim == (6LLU << 32) + 10U)
 *	// ここで時間が経過したことを想定
 *	111:get_tim(&systim)
 *	112:	[target_hrt_get_current -> 370U]
 *	113:assert(systim == (6LLU << 32) - 200U + 260U)
 *	// adj_tim(0)の動作を確認
 *		adj_tim(0)									... (C-4)(D-1)(E-6)
 *	114:	[target_hrt_get_current -> 380U]
 *	115:	[target_hrt_set_event <- HRTCNT_EMPTY]
 *	// システム時刻を確認
 *	116:get_tim(&systim)
 *	117:	[target_hrt_get_current -> 390U]
 *	118:assert(systim == (6LLU << 32) - 200U + 280U)
 *	119:END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "hrt_systim2.h"
#include "target_timer.h"

#if !defined(HRT_CONFIG1) || defined(HRT_CONFIG2)
#error Compiler option "-DHRT_CONFIG1" is missing.
#endif

#ifdef TOPPERS_SUPPORT_DRIFT
#define HRTCNT_EMPTY	TMAX_RELTIM
#else /* TOPPERS_SUPPORT_DRIFT */
#define HRTCNT_EMPTY	HRTCNT_BOUND
#endif /* TOPPERS_SUPPORT_DRIFT */

#define target_hrt_get_current	_kernel_target_hrt_get_current
#define target_hrt_set_event	_kernel_target_hrt_set_event
#define target_hrt_raise_event	_kernel_target_hrt_raise_event

void
target_hrt_raise_event(void)
{
}

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	alarm1_count = 0;

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;
	SYSTIM	systim;

	switch (++alarm1_count) {
	case 1:
		check_point(36);
		ercd = set_tim(1LLU << 32);
		check_ercd(ercd, E_CTX);

		ercd = get_tim(&systim);
		check_ercd(ercd, E_CTX);

		return;

		check_point(0);

	case 2:
		check_point(74);
		ercd = adj_tim(+1000000);
		check_ercd(ercd, E_OK);

		check_point(76);
		ercd = adj_tim(+1000000);
		check_ercd(ercd, E_OBJ);

		check_point(78);
		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	alarm2_count = 0;

void
alarm2_handler(intptr_t exinf)
{

	switch (++alarm2_count) {
	case 1:
		check_point(79);
		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RALM	ralm;
	SYSTIM	systim;

	check_point(3);
	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = set_tim(1000000LLU);
	check_ercd(ercd, E_CTX);

	ercd = get_tim(&systim);
	check_ercd(ercd, E_CTX);

	ercd = adj_tim(100);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(6);
	check_assert(systim == 40U - 10U);

	ercd = set_tim(2LLU << 32);
	check_ercd(ercd, E_OK);

	check_point(8);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(10);
	check_assert(systim == (2LLU << 32) + 10U);

	check_point(11);
	target_raise_hrt_int(1U);

	ercd = tslp_tsk(TMAX_RELTIM);
	check_ercd(ercd, E_TMOUT);

	check_point(17);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(19);
	check_assert(systim == (2LLU << 32) + 4000000050U);

	check_point(20);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(22);
	check_assert(systim == (3LLU << 32) + 50U);

	ercd = adj_tim(-1000001);
	check_ercd(ercd, E_PAR);

	ercd = adj_tim(+1000001);
	check_ercd(ercd, E_PAR);

	ercd = sta_alm(ALM1, 2000000U);
	check_ercd(ercd, E_OK);

	check_point(25);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(27);
	check_assert(systim == (3LLU << 32) + 80U);

	ercd = adj_tim(+1000000);
	check_ercd(ercd, E_OK);

	check_point(30);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(32);
	check_assert(systim == (3LLU << 32) + 1000000U + 100U);

	ercd = ref_alm(ALM1, &ralm);
	check_ercd(ercd, E_OK);

	check_point(34);
	check_assert(ralm.lefttim == 999960U);

	target_raise_hrt_int(0U);

	check_point(39);
	ercd = sta_alm(ALM1, 2000000U);
	check_ercd(ercd, E_OK);

	check_point(42);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(44);
	check_assert(systim == (3LLU << 32) + 1000000U + 1000110U);

	ercd = adj_tim(-1000000);
	check_ercd(ercd, E_OK);

	check_point(47);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(49);
	check_assert(systim == (3LLU << 32) + 1000000U + 1000120U);

	ercd = ref_alm(ALM1, &ralm);
	check_ercd(ercd, E_OK);

	check_assert(ralm.lefttim == 2999960U);

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(52);
	check_assert(systim == (3LLU << 32) + 1000000U + 1000120U);

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(54);
	check_assert(systim == (3LLU << 32) + 2000160U);

	ercd = adj_tim(-1000000);
	check_ercd(ercd, E_OK);

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(58);
	check_assert(systim == (3LLU << 32) + 2000170U);

	ercd = set_tim(4LLU << 32);
	check_ercd(ercd, E_OK);

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(61);
	check_assert(systim == (4LLU << 32));

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(63);
	check_assert(systim == (4LLU << 32));

	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(65);
	check_assert(systim == (4LLU << 32) - 999980U + 1000020U);

	ercd = sta_alm(ALM1, 1000U);
	check_ercd(ercd, E_OK);

	check_point(70);
	ercd = sta_alm(ALM2, 1000U);
	check_ercd(ercd, E_OK);

	check_point(72);
	target_raise_hrt_int(0U);

	check_point(82);
	target_raise_hrt_int(0U);

	check_point(85);
	ercd = set_tim(5LLU << 32);
	check_ercd(ercd, E_OK);

	check_point(87);
	ercd = adj_tim(200);
	check_ercd(ercd, E_OK);

	check_point(90);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(92);
	check_assert(systim == (5LLU << 32) + 200U + 20U);

	check_point(93);
	ercd = set_tim(6LLU << 32);
	check_ercd(ercd, E_OK);

	check_point(95);
	ercd = adj_tim(-200);
	check_ercd(ercd, E_OK);

	check_point(98);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(100);
	check_assert(systim == (6LLU << 32) + 10U);

	ercd = adj_tim(-1000000);
	check_ercd(ercd, E_OK);

	check_point(103);
	ercd = adj_tim(-1000000);
	check_ercd(ercd, E_OBJ);

	check_point(105);
	ercd = adj_tim(+1000000);
	check_ercd(ercd, E_OK);

	check_point(108);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(110);
	check_assert(systim == (6LLU << 32) + 10U);

	check_point(111);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(113);
	check_assert(systim == (6LLU << 32) - 200U + 260U);

	ercd = adj_tim(0);
	check_ercd(ercd, E_OK);

	check_point(116);
	ercd = get_tim(&systim);
	check_ercd(ercd, E_OK);

	check_point(118);
	check_assert(systim == (6LLU << 32) - 200U + 280U);

	check_finish(119);
	check_point(0);
}

static uint_t	target_hrt_get_current_count = 0;

HRTCNT
target_hrt_get_current(void)
{

	switch (++target_hrt_get_current_count) {
	case 1:
		test_start(__FILE__);

		check_point(1);
		return(10U);

		check_point(0);

	case 2:
		check_point(5);
		return(40U);

		check_point(0);

	case 3:
		check_point(7);
		return(50U);

		check_point(0);

	case 4:
		check_point(9);
		return(60U);

		check_point(0);

	case 5:
		check_point(12);
		return(70U);

		check_point(0);

	case 6:
		check_point(14);
		return(4000000080U);

		check_point(0);

	case 7:
		check_point(15);
		return(4000000090U);

		check_point(0);

	case 8:
		check_point(18);
		return(4000000100U);

		check_point(0);

	case 9:
		check_point(21);
		return(100U);

		check_point(0);

	case 10:
		check_point(23);
		return(120U);

		check_point(0);

	case 11:
		check_point(26);
		return(130U);

		check_point(0);

	case 12:
		check_point(28);
		return(140U);

		check_point(0);

	case 13:
		check_point(31);
		return(150U);

		check_point(0);

	case 14:
		check_point(33);
		return(160U);

		check_point(0);

	case 15:
		check_point(35);
		return(1000130U);

		check_point(0);

	case 16:
		check_point(37);
		return(1000140U);

		check_point(0);

	case 17:
		check_point(40);
		return(1000150U);

		check_point(0);

	case 18:
		check_point(43);
		return(1000160U);

		check_point(0);

	case 19:
		check_point(45);
		return(1000170U);

		check_point(0);

	case 20:
		check_point(48);
		return(1000180U);

		check_point(0);

	case 21:
		check_point(50);
		return(1000190U);

		check_point(0);

	case 22:
		check_point(51);
		return(1500200U);

		check_point(0);

	case 23:
		check_point(53);
		return(2000210U);

		check_point(0);

	case 24:
		check_point(55);
		return(2000220U);

		check_point(0);

	case 25:
		check_point(57);
		return(2000230U);

		check_point(0);

	case 26:
		check_point(59);
		return(2000240U);

		check_point(0);

	case 27:
		check_point(60);
		return(2000250U);

		check_point(0);

	case 28:
		check_point(62);
		return(2500250U);

		check_point(0);

	case 29:
		check_point(64);
		return(3000260U);

		check_point(0);

	case 30:
		check_point(66);
		return(3000270U);

		check_point(0);

	case 31:
		check_point(68);
		return(3000270U);

		check_point(0);

	case 32:
		check_point(71);
		return(3000280U);

		check_point(0);

	case 33:
		check_point(73);
		return(3001280U);

		check_point(0);

	case 34:
		check_point(75);
		return(3001290U);

		check_point(0);

	case 35:
		check_point(77);
		return(3001300U);

		check_point(0);

	case 36:
		check_point(80);
		return(3001400U);

		check_point(0);

	case 37:
		check_point(83);
		return(-100U);

		check_point(0);

	case 38:
		check_point(86);
		return(-90U);

		check_point(0);

	case 39:
		check_point(88);
		return(-80U);

		check_point(0);

	case 40:
		check_point(91);
		return(-70U);

		check_point(0);

	case 41:
		check_point(94);
		return(110U);

		check_point(0);

	case 42:
		check_point(96);
		return(120U);

		check_point(0);

	case 43:
		check_point(99);
		return(130U);

		check_point(0);

	case 44:
		check_point(101);
		return(140U);

		check_point(0);

	case 45:
		check_point(104);
		return(150U);

		check_point(0);

	case 46:
		check_point(106);
		return(160U);

		check_point(0);

	case 47:
		check_point(109);
		return(170U);

		check_point(0);

	case 48:
		check_point(112);
		return(370U);

		check_point(0);

	case 49:
		check_point(114);
		return(380U);

		check_point(0);

	case 50:
		check_point(117);
		return(390U);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
	return(0U);
}

static uint_t	target_hrt_set_event_count = 0;

void
target_hrt_set_event(HRTCNT hrtcnt)
{

	switch (++target_hrt_set_event_count) {
	case 1:
		check_point(2);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 2:
		check_point(13);
		check_assert(hrtcnt == 4000000001U);

		return;

		check_point(0);

	case 3:
		check_point(16);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 4:
		check_point(24);
		check_assert(hrtcnt == 2000001U);

		return;

		check_point(0);

	case 5:
		check_point(29);
		check_assert(hrtcnt == 999981U);

		return;

		check_point(0);

	case 6:
		check_point(38);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 7:
		check_point(41);
		check_assert(hrtcnt == 2000001U);

		return;

		check_point(0);

	case 8:
		check_point(46);
		check_assert(hrtcnt == 2999981U);

		return;

		check_point(0);

	case 9:
		check_point(56);
		check_assert(hrtcnt == 2999931U);

		return;

		check_point(0);

	case 10:
		check_point(67);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 11:
		check_point(69);
		check_assert(hrtcnt == 1001U);

		return;

		check_point(0);

	case 12:
		check_point(81);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 13:
		check_point(84);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 14:
		check_point(89);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 15:
		check_point(97);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 16:
		check_point(102);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 17:
		check_point(107);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	case 18:
		check_point(115);
		check_assert(hrtcnt == HRTCNT_EMPTY);

		return;

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}
