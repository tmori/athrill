/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2006-2012 by Embedded and Real-Time Systems Laboratory
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
 *  @(#) $Id: test_tex1.c 2419 2012-11-11 06:31:18Z ertl-hiro $
 */

/* 
 *		タスク例外処理に関するテスト(1)
 *
 * 【テストの目的】
 *
 *  タスクからタスク例外処理ルーチンを起動する処理を網羅的にテストする．
 *
 * 【テスト項目】
 *
 *	(A) ras_texのエラー検出
 *		(A-1) 対象タスクが休止状態
 *		(A-2) 対象タスクのタスク例外処理ルーチンが定義されていない
 *	(B) dis_texのエラー検出
 *		(B-1) 自タスクのタスク例外処理ルーチンが定義されていない
 *	(C) ena_texのエラー検出
 *		(C-1) 自タスクのタスク例外処理ルーチンが定義されていない
 *	(D) ref_texのエラー検出
 *		(D-1) 対象タスクが休止状態
 *		(D-2) 対象タスクのタスク例外処理ルーチンが定義されていない
 *	(E) ras_texの正常処理
 *		(E-1) 対象タスクが自タスク，タスク例外処理許可，かつ割込み優先
 *			  度マスク全解除状態で，すぐに実行開始
 *		(E-2) 対象タスクが自タスクでない
 *		(E-3) 対象タスクが自タスクだが，タスク例外処理禁止
 *		(E-4) 対象タスクが自タスクでタスク例外処理許可だが，割込み優先
 *			  度マスクが全解除でない
 *	(F) ena_texの正常処理
 *		(F-1) タスク例外処理要求があり，かつ割込み優先度マスク全解除状
 *			  態で，すぐに実行開始
 *		(F-2) タスク例外処理要求がない
 *		(F-3) タスク例外処理要求があるが，割込み優先度マスクが全解除で
 *			  ない
 *	(G) chg_ipmの正常処理
 *		(G-1) タスク例外処理要求があり，かつタスク例外処理許可で，すぐ
 *			  に実行開始
 *		(G-2) タスク例外処理要求がない
 *		(G-3) タスク例外処理要求があるが，タスク例外処理禁止
 *		(G-4) タスク例外処理要求があるが，割込み優先度マスクが全解除で
 *			  ない
 *	(H) タスクディスパッチャによる起動
 *		(H-1) ディスパッチ後のタスクがタスク例外許可でタスク例外処理要
 *			  求あり
 *	(I) タスク例外処理ルーチンからのリターンによる起動（連続起動）
 *	(J) タスク例外処理ルーチンからの戻り時による状態復帰
 *		(J-1) タスクに戻ってくる時
 *		(J-2) タスク例外処理ルーチンが連続起動される時
 *	(K) タスク例外処理ルーチンの多重起動
 *	(L) タスク例外処理ルーチンからの戻り時のタスク切換え
 *
 * 【使用リソース】
 *
 *	TASK1: メインのタスク．自タスクに対してタスク例外処理を要求する
 *	TASK2: 他タスクに対してタスク例外処理を要求する対象タスク
 *	TASK3: タスク例外処理ルーチンが定義されていないタスク
 *	TASK4: 休止状態のタスク
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：10）==
 *	1:	初期状態のチェック
 *		ref_tex(TSK_SELF, &rtex)
 *		ras_tex(TASK3, 0x0001) -> E_OBJ		... (A-2)
 *		ras_tex(TASK4, 0x0001) -> E_OBJ		... (A-1)
 *		ref_tex(TASK3, &rtex) -> E_OBJ		... (D-2)
 *		ref_tex(TASK4, &rtex) -> E_OBJ		... (D-1)
 *	2:	ena_tex()							... (F-2)
 *		ref_tex(TSK_SELF, &rtex)
 *	3:	ras_tex(TSK_SELF, 0x0001)			... (E-1)
 *	== TASK1タスク例外処理ルーチン（1回目）==
 *	4:	初期状態のチェック
 *	5:	dis_dsp() ... 4つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()
 *		loc_cpu()
 *		リターン
 *	== TASK1（続き）==
 *	6:	戻ってきた状態のチェック			... (J-1)
 *	7:	dis_dsp() ... ディスパッチ禁止，タスク例外処理禁止
 *		dis_tex()
 *	8:	ras_tex(TASK1, 0x0002)				... (E-3)
 *		ref_tex(TSK_SELF, &rtex)
 *	9:	ena_tex()							... (F-1)
 *	== TASK1タスク例外処理ルーチン（2回目）==
 *	10:	初期状態のチェック
 *	11:	ras_tex(TASK1, 0x0001)				... (E-3)
 *		ras_tex(TASK1, 0x0002)				... (E-3)
 *	12:	ena_dsp() ... 3つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		loc_cpu()
 *		リターン							... (I)
 *	== TASK1タスク例外処理ルーチン（3回目）==
 *	13:	初期状態のチェック					... (J-2)
 *	14:	ena_dsp() ... ディスパッチ許可，タスク例外許可
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()
 *		chg_ipm(TIPM_ENAALL)				... (G-2)
 *		chg_ipm(TMAX_INTPRI)
 *	15: ras_tex(TSK_SELF, 0x0004)			... (E-4)
 *		chg_ipm(TMAX_INTPRI)				... (G-4)
 *		dis_tex()
 *		chg_ipm(TIPM_ENAALL)				... (G-3)
 *		chg_ipm(TMAX_INTPRI)
 *		ena_tex()							... (F-3)
 *	16:	chg_ipm(TIPM_ENAALL)				... (G-1)(K)
 *	== TASK1タスク例外処理ルーチン（4回目）==
 *	17:	初期状態のチェック
 *	18:	dis_dsp() ... 3つの状態をそれぞれ変化させる
 *		chg_ipm(TMAX_INTPRI)
 *		loc_cpu()
 *		リターン
 *	== TASK1タスク例外処理ルーチン（3回目続き）==
 *	19:	戻ってきた状態のチェック
 *		リターン
 *	== TASK1（続き）==
 *	20:	戻ってきた状態のチェック			... (J-1)
 *	21: ena_dsp()
 *		rot_rdq(TPRI_SELF)
 *	== TASK2（優先度：10）	==
 *	22:	初期状態のチェック
 *	23:	ena_tex()
 *		rot_rdq(TPRI_SELF)
 *	== TASK3（優先度：10）	==
 *	24:	初期状態のチェック
 *	25:	ena_tex() -> E_OBJ					... (C-1)
 *	26:	dis_tex() -> E_OBJ					... (B-1)
 *		ext_tsk()
 *	== TASK1（続き）==
 *	27: ras_tex(TASK2, 0x0001)				... (E-2)
 *		ref_tex(TASK2, &rtex)
 *	28:	rot_rdq(TPRI_SELF)					... (H-1)
 *	== TASK2タスク例外処理ルーチン（1回目）==
 *	29:	初期状態のチェック
 *		リターン
 *	== TASK2（続き）==
 *	30: ras_tex(TSK_SELF, 0x0002)
 *	== TASK2タスク例外処理ルーチン（2回目）==
 *	31:	初期状態のチェック
 *	32:	dis_dsp()
 *		rot_rdq(TPRI_SELF)
 *	33:	リターン							... (L)
 *	== TASK1（続き）==
 *	34:	リターン（タスク終了）
 *	== TASK2（続き）==
 *	35:	テスト終了
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_tex1.h"

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	switch (texptn) {
	case 0x0001:
		check_point(4);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		/*
		 *  ディスパッチ禁止，割込み優先度マスク変更，タスク例外処理許可
		 */
		check_point(5);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = ena_tex();
		check_ercd(ercd, E_OK);
		ercd = loc_cpu();
		check_ercd(ercd, E_OK);
		check_state(false, true, TMAX_INTPRI, true, true, false);
		break;

	case 0x0002:
		check_point(10);
		check_state(false, false, TIPM_ENAALL, true, true, true);

		/*
		 *  タスク例外処理を要求
		 */
		check_point(11);
		ercd = ras_tex(TSK_SELF, 0x0001);
		check_ercd(ercd, E_OK);
		ercd = ras_tex(TSK_SELF, 0x0002);
		check_ercd(ercd, E_OK);

		/*
		 *  ディスパッチ許可，割込み優先度マスク変更，CPUロック
		 */
		check_point(12);
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = loc_cpu();
		check_ercd(ercd, E_OK);
		check_state(false, true, TMAX_INTPRI, false, true, true);
		break;

	case 0x0003:
		check_point(13);
		check_state(false, false, TIPM_ENAALL, true, true, true);

		/*
		 *  ディスパッチ許可，割込み優先度マスク変更，タスク例外許可
		 */
		check_point(14);
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = ena_tex();
		check_ercd(ercd, E_OK);
		check_state(false, false, TMAX_INTPRI, false, true, false);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		/*
		 *  タスク例外処理を要求
		 */
		check_point(15);
		ercd = ras_tex(TSK_SELF, 0x0004);

		ercd = dis_tex();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = ena_tex();
		check_ercd(ercd, E_OK);

		check_point(16);
		ercd = chg_ipm(TIPM_ENAALL);
		/* ここでタスク例外処理ルーチンが動作する */
		check_ercd(ercd, E_OK);

		/*
		 *  タスク例外処理からのリターンにより元の状態に戻っていること
		 *  をチェック
		 */
		check_point(19);
		check_state(false, false, TIPM_ENAALL, false, false, false);
		break;

	case 0x0004:
		check_point(17);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		/*
		 *  ディスパッチ禁止，割込み優先度マスク変更，CPUロック
		 */
		check_point(18);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);
		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);
		ercd = loc_cpu();
		check_ercd(ercd, E_OK);
		check_state(false, true, TMAX_INTPRI, true, true, true);
		break;

	default:
		check_point(0);
		break;
	}
}

void
task1(intptr_t exinf)
{
	ER		ercd;
	T_RTEX	rtex;

	/*
	 *  初期状態のチェック
	 */
	check_point(1);
	check_state(false, false, TIPM_ENAALL, false, false, true);
	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);
	check_assert((rtex.texstat & TTEX_DIS) != 0);
	check_assert(rtex.pndptn == 0);

	/*
	 *  ras_texのエラー検出
	 */
	ercd = ras_tex(TASK3, 0x0001);
	check_ercd(ercd, E_OBJ);
	ercd = ras_tex(TASK4, 0x0001);
	check_ercd(ercd, E_OBJ);

	/*
	 *  ref_texのエラー検出
	 */
	ercd = ref_tex(TASK3, &rtex);
	check_ercd(ercd, E_OBJ);
	ercd = ref_tex(TASK4, &rtex);
	check_ercd(ercd, E_OBJ);

	/*
	 *  タスク例外処理の許可
	 */
	check_point(2);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, false, false, false);
	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);
	check_assert((rtex.texstat & TTEX_ENA) != 0);
	check_assert(rtex.pndptn == 0);

	/*
	 *  タスク例外処理を要求
	 */
	check_point(3);
	ercd = ras_tex(TSK_SELF, 0x0001);
	/* ここでタスク例外処理ルーチンが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク例外処理からのリターンにより元の状態に戻っていることを
	 *  チェック
	 */
	check_point(6);
	check_state(false, false, TIPM_ENAALL, false, false, false);

	/*
	 *  ディスパッチ禁止，タスク例外処理禁止
	 */
	check_point(7);
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ercd = dis_tex();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, true, true, true);

	/*
	 *  タスク例外処理を要求
	 */
	check_point(8);
	ercd = ras_tex(TASK1, 0x0002);
	check_ercd(ercd, E_OK);
	ercd = ref_tex(TSK_SELF, &rtex);
	check_ercd(ercd, E_OK);
	check_assert((rtex.texstat & TTEX_DIS) != 0);
	check_assert(rtex.pndptn == 0x0002);

	/*
	 *  タスク例外処理を許可
	 */
	check_point(9);
	ercd = ena_tex();
	/* ここでタスク例外処理ルーチンが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク例外処理からのリターンにより元の状態に戻っていることを
	 *  チェック
	 */
	check_point(20);
	check_state(false, false, TIPM_ENAALL, true, true, false);

	/*
	 *  タスク2に切り換える
	 */
	check_point(21);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	ercd = rot_rdq(TPRI_SELF);
	/* ここで他のタスクが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク2に対してタスク例外処理を要求
	 */
	check_point(27);
	ercd = ras_tex(TASK2, 0x0001);
	check_ercd(ercd, E_OK);
	ercd = ref_tex(TASK2, &rtex);
	check_ercd(ercd, E_OK);
	check_assert((rtex.texstat & TTEX_ENA) != 0);
	check_assert(rtex.pndptn == 0x0001);

	/*
	 *  タスク2に切り換える
	 */
	check_point(28);
	ercd = rot_rdq(TPRI_SELF);
	/* ここで他のタスクが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク終了
	 */
	check_point(34);
}

void
tex_task2(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	switch (texptn) {
	case 0x0001:
		check_point(29);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		break;

	case 0x0002:
		check_point(31);
		check_state(false, false, TIPM_ENAALL, false, false, true);

		/*
		 *  ディスパッチを禁止して，タスク切換えを要求する．
		 */
		check_point(32);
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);
		ercd = rot_rdq(TPRI_SELF);
		check_ercd(ercd, E_OK);

		/*
		 *  タスク例外処理ルーチンからのリターンで，タスク切換えが発生
		 *  する．
		 */
		check_point(33);
		break;

	default:
		check_point(0);
		break;
	}
}

void
task2(intptr_t exinf)
{
	ER		ercd;

	/*
	 *  初期状態のチェック
	 */
	check_point(22);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  タスク例外処理の許可
	 */
	check_point(23);
	ercd = ena_tex();
	check_ercd(ercd, E_OK);
	check_state(false, false, TIPM_ENAALL, false, false, false);

	/*
	 *  タスク3に切り換える
	 */
	ercd = rot_rdq(TPRI_SELF);
	/* ここで他のタスクが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  タスク例外処理を要求
	 */
	check_point(30);
	ercd = ras_tex(TSK_SELF, 0x0002);
	/* ここでタスク例外処理ルーチンが動作する */
	check_ercd(ercd, E_OK);

	/*
	 *  テスト終了
	 */
	check_finish(35);
}

void
task3(intptr_t exinf)
{
	ER		ercd;

	/*
	 *  初期状態のチェック
	 */
	check_point(24);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  タスク例外処理の許可
	 */
	check_point(25);
	ercd = ena_tex();
	check_ercd(ercd, E_OBJ);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  タスク例外処理の禁止
	 */
	check_point(26);
	ercd = dis_tex();
	check_ercd(ercd, E_OBJ);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	/*
	 *  タスク終了
	 */
	ercd = ext_tsk();
	check_point(0);
}

void
task4(intptr_t exinf)
{
	check_point(0);
}

void
tex_task4(TEXPTN texptn, intptr_t exinf)
{
	check_point(0);
}
