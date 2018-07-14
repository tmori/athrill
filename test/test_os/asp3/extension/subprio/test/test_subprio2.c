/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2014-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_subprio2.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		サブ優先度機能のテスト(2)
 *
 * 【テストの目的】
 *
 *  サブ優先度を用いる優先度が2つある場合をテストする．また，chg_sprが
 *  エラーに成るケースをテストする．
 *
 * 【テスト項目】
 *
 *	(A) サブ優先度を用いる優先度がENA_SPRにより複数設定できること
 *	(B) chg_sprがエラーになる場合
 *		(B-1) 非タスクコンテキストからの呼出し［NGKI3666］
 *		(B-2) CPUロック状態からの呼出し［NGKI3667］
 *		(B-3) 不正ID番号［NGKI3669］
 *
 * 【使用リソース】
 *
 *	TASK1: 高優先度タスク，メインタスク，最初から起動
 *	TASK2: 中優先度タスク
 *	TASK3: 中優先度タスク
 *	TASK4: 低優先度タスク
 *	TASK5: 低優先度タスク
 *	中優先度と低優先度を，サブ優先度を使って優先順位を決めるように設定
 *	ALM1:  アラームハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：高）==
 *	1:	chg_spr(TASK2, 10)
 *		chg_spr(TASK3, 5)
 *		chg_spr(TASK4, 10)
 *		chg_spr(TASK5, 5)
 *		act_tsk(TASK2)
 *		act_tsk(TASK3)
 *		act_tsk(TASK4)
 *		act_tsk(TASK5)
 *		slp_tsk()
 *	== TASK3（優先度：中）==
 *	2:	ext_tsk()
 *	== TASK2（優先度：中）==
 *	3:	ext_tsk()
 *	== TASK5（優先度：低）==
 *	4:	ext_tsk()
 *	== TASK4（優先度：低）==
 *	5:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	6:	chg_spr(-1, 10) -> E_ID						... (B-3)
 *		chg_spr(TNUM_TSKID + 1, 10) -> E_ID			... (B-3)
 *		loc_cpu()
 *		chg_spr(TASK2, 10) -> E_CTX					... (B-2)
 *		unl_cpu()
 *		sta_alm(ALM1, TEST_TIME_CP) ... ALM1-1が実行開始するまで
 *		slp_tsk()
 *	== TASK4（続き）==
 *	7:	ext_tsk()
 *	== ALM1 ==
 *	8:	chg_spr(TASK2, 10) -> E_CTX					... (B-1)
 *		wup_tsk(TASK1)
 *		RETURN
 *	== TASK1（続き）==
 *	9:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_subprio2.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(8);
	ercd = chg_spr(TASK2, 10);
	check_ercd(ercd, E_CTX);

	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	check_point(1);
	ercd = chg_spr(TASK2, 10);
	check_ercd(ercd, E_OK);

	ercd = chg_spr(TASK3, 5);
	check_ercd(ercd, E_OK);

	ercd = chg_spr(TASK4, 10);
	check_ercd(ercd, E_OK);

	ercd = chg_spr(TASK5, 5);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK5);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(6);
	ercd = chg_spr(-1, 10);
	check_ercd(ercd, E_ID);

	ercd = chg_spr(TNUM_TSKID + 1, 10);
	check_ercd(ercd, E_ID);

	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = chg_spr(TASK2, 10);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	ercd = sta_alm(ALM1, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_finish(9);
	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(3);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(2);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(5);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task5(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(4);
	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}
