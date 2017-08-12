/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2010-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_rstr1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		制約タスクのテスト(1)
 *
 * 【テストの目的】
 *
 *  制約タスクの基本的な振舞いをテストする．
 *
 * 【テスト項目】
 *
 *	(A) 制約タスクが，自タスクを待ち状態にする可能性のあるサービスコー
 *      ルを呼び出した場合，E_NOSPTエラーとなる
 *		(A-1) slp_tsk［NGKI1255］
 *		(A-2) tslp_tsk［NGKI1255］
 *		(A-3) dly_tsk［NGKI1350］
 *		(A-4) wai_sem［NGKI1516］
 *		(A-5) pol_semが正しく動作する
 *		(A-6) twai_sem［NGKI1516］
 *      ※ 他にもあるが，これだけに留める
 *	(B) 制約タスクを対象として，chg_pri，wup_tsk，can_wup，rel_wai，
 *	    sus_tsk，rsm_tskを呼び出した場合，E_NOSPTエラーとなる
 *		(B-1) chg_pri［NGKI1186］
 *		(B-2) wup_tsk（タスクコンテキストから）［NGKI1266］
 *		(B-3) wup_tsk（非タスクコンテキストから）［NGKI1266］
 *		(B-4) can_wup［NGKI1279］
 *		(B-5) rel_wai（タスクコンテキストから）［NGKI1291］
 *		(B-6) rel_wai（非タスクコンテキストから）［NGKI1291］
 *		(B-7) sus_tsk［NGKI1301］
 *		(B-8) rsm_tsk［NGKI1315］
 *  (C) rot_rdqは，対象優先度を持つ実行できる状態のタスクの中で最も優先
 *      順位が高いタスクが制約タスクである場合，E_NOSPTエラーとなる
 *		(C-1) rot_rdq（タスクコンテキストから）が正しく動作する
 *		(C-2) rot_rdq（タスクコンテキストから）がE_NOSPTを返す［NGKI2690］
 *		(C-3) rot_rdq（非タスクコンテキストから）が正しく動作する
 *		(C-4) rot_rdq（非タスクコンテキストから）がE_NOSPTを返す［NGKI2690］
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，メインタスク，最初から起動
 *	TASK2: 高優先度タスク，制約タスク
 *	TASK3: 中優先度タスク，制約タスク
 *	TASK4: 中優先度タスク，制約タスク
 *	SEM1:  セマフォ
 *	ALM1:  アラームハンドラ
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	act_tsk(TASK2)
 *	== TASK2-1（優先度：高）==
 *	2:	slp_tsk() -> E_NOSPT						... (A-1)
 *		tslp_tsk(TEST_TIME_PROC) -> E_NOSPT			... (A-2)
 *		dly_tsk(TEST_TIME_PROC) -> E_NOSPT			... (A-3)
 *		wai_sem(SEM1) -> E_NOSPT					... (A-4)
 *		pol_sem(SEM1)								... (A-5)
 *		pol_sem(SEM1) -> E_TMOUT
 *		twai_sem(SEM1, TEST_TIME_PROC) -> E_NOSPT	... (A-6)
 *  	ext_tsk()
 *	== TASK1（続き）==
 *	3:	chg_pri(TASK3, HIGH_PRIORITY) -> E_NOSPT	... (B-1)
 *		wup_tsk(TASK3) -> E_NOSPT					... (B-2)
 *		can_wup(TASK3) -> E_NOSPT					... (B-4)
 *		rel_wai(TASK3) -> E_NOSPT					... (B-5)
 *		sus_tsk(TASK3) -> E_NOSPT					... (B-7)
 *		rsm_tsk(TASK3) -> E_NOSPT					... (B-8)
 *		sta_alm(ALM1, TEST_TIME_PROC) ... ALM1が実行開始するまで
 *		slp_tsk()
 *	== ALM1 ==
 *	4:	wup_tsk(TASK3) -> E_NOSPT					... (B-3)
 *		rel_wai(TASK3) -> E_NOSPT					... (B-6)
 *		wup_tsk(TASK1)
 *		act_tsk(TASK3)
 *		act_tsk(TASK4)
 *		rot_rdq(MID_PRIORITY)						... (C-3)
 *		rot_rdq(MID_PRIORITY) -> E_NOSPT			... (C-4)
 *		RETURN
 *	== TASK3-1（優先度：中）==
 *	5:	act_tsk(TASK2)
 *	== TASK2-2（優先度：高）2回め ==
 *	6:	rot_rdq(MID_PRIORITY) -> E_NOSPT			... (C-2)
 *		ext_tsk()
 *	== TASK3-1（続き）==
 *	7:	ext_tsk()
 *	== TASK4-1（優先度：中）1回め ==
 *	8:	ext_tsk()
 *	== TASK1（続き）==
 *	9:	act_tsk(TASK3)
 *		act_tsk(TASK4)
 *		act_tsk(TASK2)
 *	== TASK2-3（優先度：高）3回め ==
 *	10:	rot_rdq(MID_PRIORITY)						... (C-1)
 *		ext_tsk()
 *	== TASK3-2（優先度：中）2回め ==
 *	11:	ext_tsk()
 *	== TASK4-2（優先度：中）2回め ==
 *	12:	ext_tsk()
 *	== TASK1（続き）==
 *	13: END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_rstr1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;

	check_point(4);
	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = rel_wai(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_OK);

	ercd = rot_rdq(MID_PRIORITY);
	check_ercd(ercd, E_NOSPT);

	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;

	test_start(__FILE__);

	check_point(1);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(3);
	ercd = chg_pri(TASK3, HIGH_PRIORITY);
	check_ercd(ercd, E_NOSPT);

	ercd = wup_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = can_wup(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = rel_wai(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = sus_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = rsm_tsk(TASK3);
	check_ercd(ercd, E_NOSPT);

	ercd = sta_alm(ALM1, TEST_TIME_PROC);
	check_ercd(ercd, E_OK);

	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK4);
	check_ercd(ercd, E_OK);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_finish(13);
	check_point(0);
}

static uint_t	task2_count = 0;

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task2_count) {
	case 1:
		check_point(2);
		ercd = slp_tsk();
		check_ercd(ercd, E_NOSPT);

		ercd = tslp_tsk(TEST_TIME_PROC);
		check_ercd(ercd, E_NOSPT);

		ercd = dly_tsk(TEST_TIME_PROC);
		check_ercd(ercd, E_NOSPT);

		ercd = wai_sem(SEM1);
		check_ercd(ercd, E_NOSPT);

		ercd = pol_sem(SEM1);
		check_ercd(ercd, E_OK);

		ercd = pol_sem(SEM1);
		check_ercd(ercd, E_TMOUT);

		ercd = twai_sem(SEM1, TEST_TIME_PROC);
		check_ercd(ercd, E_NOSPT);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(6);
		ercd = rot_rdq(MID_PRIORITY);
		check_ercd(ercd, E_NOSPT);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(10);
		ercd = rot_rdq(MID_PRIORITY);
		check_ercd(ercd, E_OK);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	task3_count = 0;

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task3_count) {
	case 1:
		check_point(5);
		ercd = act_tsk(TASK2);
		check_ercd(ercd, E_OK);

		check_point(7);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(11);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}

static uint_t	task4_count = 0;

void
task4(intptr_t exinf)
{
	ER_UINT	ercd;

	switch (++task4_count) {
	case 1:
		check_point(8);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(12);
		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	default:
		check_point(0);
	}
	check_point(0);
}
