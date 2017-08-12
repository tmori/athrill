/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2008-2017 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_int1.c 796 2017-07-19 14:21:08Z ertl-hiro $
 */

/* 
 *		割込み管理機能のテスト(1)
 *
 * 【テストの目的】
 *
 *  割込み要求ラインに対するサービスコール（dis_int，ena_int，clr_int，
 *  ras_int，prb_int）の基本動作をテストする．
 *
 * 【制限事項】
 *
 *  このテストを実施するには，dis_int，ena_int，clr_int，ras_int，
 *  prb_intがサポートされており，それらを行える割込み要求ラインがある
 *  ことが必要である．
 *
 * 【テスト項目】
 *
 *	(A) dis_intのテスト
 *		(A-1) dis_intにより，割込み要求がマスクされる
 *	(B) ena_intのテスト
 *		(B-1) ena_intにより，割込み要求がマスク解除される
 *	(C) clr_intのテスト
 *		(C-1) clr_intにより，割込み要求がクリアされる
 *	(D) ras_intのテスト
 *		(D-1) ras_intにより，割込みが要求される
 *	(E) prb_intのテスト
 *		(E-1) 割込み要求がある時は，trueを返す
 *		(E-2) 割込み要求がない時は，falseを返す
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *  ISR1:  割込みサービスルーチン
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	ras_int(INTNO1)						... (D-1)
 *	== ISR1-1 ==
 *	2:	DO(intno1_clear())
 *		RETURN
 *	== TASK1（続き）==
 *	3:	dis_int(INTNO1)						... (A-1)
 *		prb_int(INTNO1) -> false			... (E-2)
 *		ras_int(INTNO1)						... (D-1)
 *		prb_int(INTNO1) -> true				... (E-1)
 *	4:	ena_int(INTNO1)						... (B-1)
 *	== ISR1-2 ==
 *	5:	DO(intno1_clear())
 *		RETURN
 *	== TASK1（続き）==
 *	6:	DO(CHECK_SUPPORT_CLR_INT)
 *		dis_int(INTNO1)						... (A-1)
 *		prb_int(INTNO1) -> false			... (E-2)
 *		ras_int(INTNO1)						... (D-1)
 *		prb_int(INTNO1) -> true				... (E-1)
 *		clr_int(INTNO1)						... (C-1)
 *		prb_int(INTNO1) -> false			... (E-2)
 *	7:	ena_int(INTNO1)						... (B-1)
 *	8:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_int1.h"

/*
 *  clr_intがサポートされていない場合に，途中で終了させるための処理
 *
 *  CHECK_SUPPORT_CLR_INTに置かれているチェックポイントの番号が変わっ
 *  た場合には，以下のコード中のチェックポイント番号の修正が必要である．
 */
#ifdef TOPPERS_SUPPORT_CLR_INT
#define CHECK_SUPPORT_CLR_INT
#else /* TOPPERS_SUPPORT_CLR_INT */
#define CHECK_SUPPORT_CLR_INT	check_finish(7)
#define clr_int(intno)	(E_OK)
#endif /* TOPPERS_SUPPORT_CLR_INT */

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

static uint_t	isr1_count = 0;

void
isr1(intptr_t exinf)
{

	switch (++isr1_count) {
	case 1:
		check_point(2);
		intno1_clear();

		return;

		check_point(0);

	case 2:
		check_point(5);
		intno1_clear();

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

	test_start(__FILE__);

	check_point(1);
	ercd = ras_int(INTNO1);
	check_ercd(ercd, E_OK);

	check_point(3);
	ercd = dis_int(INTNO1);
	check_ercd(ercd, E_OK);

	ercd = prb_int(INTNO1);
	check_ercd(ercd, false);

	ercd = ras_int(INTNO1);
	check_ercd(ercd, E_OK);

	ercd = prb_int(INTNO1);
	check_ercd(ercd, true);

	check_point(4);
	ercd = ena_int(INTNO1);
	check_ercd(ercd, E_OK);

	check_point(6);
	CHECK_SUPPORT_CLR_INT;

	ercd = dis_int(INTNO1);
	check_ercd(ercd, E_OK);

	ercd = prb_int(INTNO1);
	check_ercd(ercd, false);

	ercd = ras_int(INTNO1);
	check_ercd(ercd, E_OK);

	ercd = prb_int(INTNO1);
	check_ercd(ercd, true);

	ercd = clr_int(INTNO1);
	check_ercd(ercd, E_OK);

	ercd = prb_int(INTNO1);
	check_ercd(ercd, false);

	check_point(7);
	ercd = ena_int(INTNO1);
	check_ercd(ercd, E_OK);

	check_finish(8);
	check_point(0);
}
