/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2015-2016 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_dtq1.c 738 2016-04-05 14:19:24Z ertl-hiro $
 */

/* 
 *		データキュー機能のテスト(1)
 *
 * 【テストの目的】
 *
 *	snd_dtqの振舞いのテスト［NGKI1718］
 *	  ・snd_dtqに関するすべての要求をテストする．
 *	  ・snd_dtq関数のC1カバレッジを達成する．
 *	snd_dtqが呼び出すデータキュー機能内部関数のテスト
 *	  ・send_data関数のC1カバレッジを達成する．
 *	  ・enqueue_data関数のC1カバレッジを達成する．
 *	rcv_dtqの振舞いのテスト［NGKI1751］
 *	  ・rcv_dtqに関するすべての要求をテストする．
 *	  ・rcv_dtq関数のC1カバレッジを達成する．
 *	rcv_dtqが呼び出すデータキュー機能内部関数のテスト
 *	  ・receive_data関数のC1カバレッジを達成する．
 *	  ・dequeue_data関数のC1カバレッジを達成する．
 *
 * 【テスト項目】
 *
 *	(A) snd_dtqのエラー検出
 *		(A-1) 非タスクコンテキストからの呼出し［NGKI1722］
 *		(A-2) CPUロック状態からの呼出し［NGKI1724］
 *		(A-3) ディスパッチ保留状態からの呼出し［NGKI1725］
 *			(A-3-1) ディスパッチ禁止状態
 *			(A-3-2) 割込み優先度マスク全解除でない状態
 *		(A-4) dtqidが不正［NGKI1727］
 *			(A-4-1) 小さすぎる
 *			(A-4-2) 大きすぎる
 *		(A-5) 待ち状態の強制解除［NGKI1732］
 *		(A-6) タスクの終了要求［NGKI3459］
 *			(A-6-1) 呼び出し時点で終了要求フラグがセット
 *			(A-6-2) 待ち状態になった後に終了要求
 *		(A-7) 待ちオブジェクトの削除または再初期化［NGKI1733］
 *	  ※ASPカーネルに適用されない要求：
 *			［NGKI1726］［NGKI1729］［NGKI1730］
 *	(B) snd_dtqの正常系処理
 *		(B-1) 受信待ち行列中のタスクを待ち解除［NGKI1734］
 *		(B-2) 待ち解除されたタスクにE_OKが返る［NGKI1735］
 *		(B-3) データをFIFO順で管理領域に格納［NGKI1736］
 *			(B-3-1) 管理領域が空の時
 *			(B-3-2) 管理領域が空でない時
 *		(B-4) 管理領域にスペースがなく送信待ち状態に［NGKI1737］
 *	(C) snd_dtqのC1カバレッジ
 *	  ※ここまでのテストで有効なもの
 *			(A-1)(A-2)(A-3)(A-4-1)(A-4-2)(A-6-1)(B-1)(B-4)
 *			※(B-1)は(B-3)に代えてもよい
 *	  ※(B-1)の分解が必要
 *		(C-1) 受信待ちタスクを待ち解除してタスク切換えが発生
 *		(C-2) 受信待ちタスクを待ち解除してタスク切換えが発生しない
 *	(D) send_datqのC1カバレッジ
 *	  ※ここまでのテストで有効なもの（不足はない）
 *			(B-1)(B-3)(B-4)
 *	(E) enqueue_dataのC1カバレッジ
 *	  ※(B-3)の分解が必要
 *		(E-1) 管理領域が循環しない（末尾に到達しない）
 *		(E-2) 管理領域が循環（末尾に到達）
 *	(F) rcv_dtqのエラー検出
 *		(F-1) 非タスクコンテキストからの呼出し［NGKI1754］
 *		(F-2) CPUロック状態からの呼出し［NGKI1755］
 *		(F-3) ディスパッチ保留状態からの呼出し［NGKI1756］
 *			(F-3-1) ディスパッチ禁止状態
 *			(F-3-2) 割込み優先度マスク全解除でない状態
 *		(F-4) dtqidが不正［NGKI1758］
 *			(F-4-1) 小さすぎる
 *			(F-4-2) 大きすぎる
 *		(F-5) 待ち状態の強制解除［NGKI1764］
 *		(F-6) タスクの終了要求［NGKI3460］
 *			(F-6-1) 呼び出し時点で終了要求フラグがセット
 *			(F-6-2) 待ち状態になった後に終了要求
 *		(F-7) 待ちオブジェクトの削除または再初期化［NGKI1765］
 *	  ※ASPカーネルに適用されない要求：
 *			［NGKI1757］［NGKI1760］［NGKI1761］［NGKI1762］
 *	(G) rcv_dtqの正常系処理
 *		(G-1) 受信したデータはp_dataが指すメモリ領域に［NGKI3421］
 *		(G-2) 管理領域の先頭のデータを受信［NGKI1766］
 *		(G-3) 送信待ち行列中のタスクを待ち解除［NGKI1767］
 *		(G-4) 待ち解除されたタスクにE_OKが返る［NGKI1768］
 *		(G-5) 送信待ち行列中のタスクの送信データを受信［NGKI1769］
 *		(G-6) 送信待ち行列中のタスクを待ち解除［NGKI3422］
 *		(G-7) 待ち解除されたタスクにE_OKが返る［NGKI1770］
 *		(G-8) 受信すべきデータがなく受信待ち状態に［NGKI1771］
 *	  ※(G-5)(G-6)(G-7)は格納できるデータ数が0の場合のみ発生する条件
 *	(H) rcv_dtqのC1カバレッジ
 *	  ※ここまでのテストで有効なもの
 *			(F-1)(F-2)(F-3)(F-4-1)(F-4-2)(F-6-1)(G-2)(G-8)
 *			※(G-2)は(G-5)に代えてもよい
 *	  ※(G-2)の分解が必要
 *		(H-1) 送信待ちタスクを待ち解除してタスク切換えが発生
 *		(H-2) タスク切換えが発生しない
 *	  ※(G-8)の分解が必要
 *		(H-3) 待ち状態から正常終了（E_OK）
 *		(H-4) 待ち状態からエラー終了（E_OK以外） → (A-5)でカバー
 *	(I) receive_datqのC1カバレッジ
 *	  ※ここまでのテストで有効なもの
 *			(G-2)(G-3)(G-5)(G-8)
 *	  ※(G-2)の分解が必要
 *		(I-1) 送信待ち行列が空
 *		(I-2) 送信待ち行列が空でない → (G-3)でカバー
 *	(J) dequeue_dataのC1カバレッジ
 *	  ※(G-2)の分解が必要
 *		(J-1) 管理領域が循環しない（末尾に到達しない）
 *		(J-2) 管理領域が循環（末尾に到達）
 *
 * 【使用リソース】
 *
 *	TASK1: 中優先度タスク，TA_ACT属性
 *	TASK2: 高優先度タスク
 *	TASK3: 低優先度タスク
 *	ALM1:  アラームハンドラ
 *  DTQ1:  TA_NULL属性，格納できるデータ数2
 *  DTQ2:  TA_NULL属性，格納できるデータ数0
 *  DTQ3:  TA_TPRI属性，格納できるデータ数2 … 使っていない
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：中）==
 *	1:	ref_dtq(DTQ1, &rdtq)
 *		assert(rdtq.stskid == TSK_NONE)
 *		assert(rdtq.rtskid == TSK_NONE)
 *		assert(rdtq.sdtqcnt == 0)
 *		loc_cpu()
 *		snd_dtq(DTQ1, DATA0) -> E_CTX					... (A-2)
 *		rcv_dtq(DTQ1, &data) -> E_CTX					... (F-2)
 *		unl_cpu()
 *		dis_dsp()
 *		snd_dtq(DTQ1, DATA0) -> E_CTX					... (A-3-1)
 *		rcv_dtq(DTQ1, &data) -> E_CTX					... (F-3-1)
 *		ena_dsp()
 *		chg_ipm(TMAX_INTPRI)
 *		snd_dtq(DTQ1, DATA0) -> E_CTX					... (A-3-2)
 *		rcv_dtq(DTQ1, &data) -> E_CTX					... (F-3-2)
 *		chg_ipm(TIPM_ENAALL)
 *		snd_dtq(0, DATA0) -> E_ID						... (A-4-1)
 *		rcv_dtq(0, &data) -> E_ID						... (F-4-1)
 *		snd_dtq(TNUM_DTQID+1, DATA0) -> E_ID			... (A-4-2)
 *		rcv_dtq(TNUM_DTQID+1, &data) -> E_ID			... (F-4-2)
 *		act_tsk(TASK3)
 *	2:	act_tsk(TASK2)
 *	== TASK2（優先度：高）==
 *	3:	rcv_dtq(DTQ1, &data)							... (G-8)(H-3)
 *	== TASK1（続き）==
 *	4:	snd_dtq(DTQ1, DATA1)							... (B-1)(C-1)
 *	== TASK2（続き）==									... (B-2)
 *	5:	assert(data == DATA1)							... (G-1)
 *	6:	slp_tsk()
 *	== TASK1（続き）==
 *	7:	rcv_dtq(DTQ1, &data)
 *	== TASK3（優先度：低）==
 *	8:	wup_tsk(TASK2)
 *	== TASK2（続き）==
 *	9:	snd_dtq(DTQ1, DATA2)							... (B-1)(C-2)
 *	10:	slp_tsk()
 *	== TASK1（続き）==
 *	11:	assert(data == DATA2)
 *		snd_dtq(DTQ1, DATA3)							... (B-3-1)(E-1)
 *		ref_dtq(DTQ1, &rdtq)
 *		assert(rdtq.stskid == TSK_NONE)
 *		assert(rdtq.rtskid == TSK_NONE)
 *		assert(rdtq.sdtqcnt == 1)
 *		snd_dtq(DTQ1, DATA4)							... (B-3-2)(E-2)
 *		ref_dtq(DTQ1, &rdtq)
 *		assert(rdtq.stskid == TSK_NONE)
 *		assert(rdtq.rtskid == TSK_NONE)
 *		assert(rdtq.sdtqcnt == 2)
 *		snd_dtq(DTQ1, DATA5)							... (B-4)
 *	== TASK3（続き）==
 *	12:	rcv_dtq(DTQ1, &data)							... (G-2)(G-3)(H-1)(J-1)
 *	== TASK1（続き）==									... (G-4)
 *	13:	slp_tsk()
 *	== TASK3（続き）==
 *	14:	assert(data == DATA3)
 *		rcv_dtq(DTQ1, &data)							... (G-2)(H-2)(I-1)(J-2)
 *		assert(data == DATA4)
 *		rcv_dtq(DTQ1, &data)
 *		assert(data == DATA5)
 *	15:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	16:	snd_dtq(DTQ2, DATA1)
 *	== TASK3（続き）==
 *	17:	rcv_dtq(DTQ2, &data)							... (G-5)(G-6)
 *	== TASK1（続き）==									... (G-7)
 *	18:	slp_tsk()
 *	== TASK3（続き）==
 *	19:	assert(data == DATA1)
 *	20:	sta_alm(ALM1, TEST_TIME_CP) ... ALM1が実行開始するまで
 *	21:	rcv_dtq(DTQ2, &data)
 *	== ALM1 ==
 *	22:	snd_dtq(DTQ2, DATA0) -> E_CTX					... (A-1)
 *		rcv_dtq(DTQ2, &data) -> E_CTX					... (F-1)
 *		psnd_dtq(DTQ2, DATA2)
 *	23:	RETURN
 *	== TASK3（続き）==
 *	24:	assert(data == DATA2)
 *	25:	wup_tsk(TASK1)
 *	== TASK1（続き）==
 *	26:	snd_dtq(DTQ2, DATA0) -> E_RLWAI					... (A-5)
 *	== TASK3（続き）==
 *	27:	rel_wai(TASK1)
 *	== TASK1（続き）==
 *	28:	snd_dtq(DTQ2, DATA0) -> E_DLT					... (A-7)
 *	== TASK3（続き）==
 *	29:	ini_dtq(DTQ2)
 *	== TASK1（続き）==
 *	30:	dis_ter()
 *	31:	snd_dtq(DTQ2, DATA0) -> E_RASTER				... (A-6-2)
 *	== TASK3（続き）==
 *	32:	ras_ter(TASK1)
 *	== TASK1（続き）==
 *	33:	snd_dtq(DTQ2, DATA0) -> E_RASTER				... (A-6-1)
 *	34:	wup_tsk(TASK2)
 *	== TASK2（続き）==
 *	35:	rcv_dtq(DTQ2, &data) -> E_RLWAI					... (F-5)
 *	== TASK1（続き）==
 *	36:	ena_ter()
 *	== TASK3（続き）==
 *	37:	rel_wai(TASK2)
 *	== TASK2（続き）==
 *	38:	rcv_dtq(DTQ2, &data) -> E_DLT					... (F-7)
 *	== TASK3（続き）==
 *	39:	ini_dtq(DTQ2)
 *	== TASK2（続き）==
 *	40:	dis_ter()
 *	41:	rcv_dtq(DTQ2, &data) -> E_RASTER				... (F-6-2)
 *	== TASK3（続き）==
 *	42:	ras_ter(TASK2)
 *	== TASK2（続き）==
 *	43:	rcv_dtq(DTQ2, &data) -> E_RASTER				... (F-6-1)
 *	44:	ena_ter()
 *	== TASK3（続き）==
 *	45:	END
 */

#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/test_svc.h"
#include "kernel_cfg.h"
#include "test_dtq1.h"

/* DO NOT DELETE THIS LINE -- gentest depends on it. */

void
alarm1_handler(intptr_t exinf)
{
	ER_UINT	ercd;
	intptr_t	data;

	check_point(22);
	ercd = snd_dtq(DTQ2, DATA0);
	check_ercd(ercd, E_CTX);

	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_CTX);

	ercd = psnd_dtq(DTQ2, DATA2);
	check_ercd(ercd, E_OK);

	check_point(23);
	return;

	check_point(0);
}

void
task1(intptr_t exinf)
{
	ER_UINT	ercd;
	T_RDTQ	rdtq;
	intptr_t	data;

	test_start(__FILE__);

	check_point(1);
	ercd = ref_dtq(DTQ1, &rdtq);
	check_ercd(ercd, E_OK);

	check_assert(rdtq.stskid == TSK_NONE);

	check_assert(rdtq.rtskid == TSK_NONE);

	check_assert(rdtq.sdtqcnt == 0);

	ercd = loc_cpu();
	check_ercd(ercd, E_OK);

	ercd = snd_dtq(DTQ1, DATA0);
	check_ercd(ercd, E_CTX);

	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_CTX);

	ercd = unl_cpu();
	check_ercd(ercd, E_OK);

	ercd = dis_dsp();
	check_ercd(ercd, E_OK);

	ercd = snd_dtq(DTQ1, DATA0);
	check_ercd(ercd, E_CTX);

	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_CTX);

	ercd = ena_dsp();
	check_ercd(ercd, E_OK);

	ercd = chg_ipm(TMAX_INTPRI);
	check_ercd(ercd, E_OK);

	ercd = snd_dtq(DTQ1, DATA0);
	check_ercd(ercd, E_CTX);

	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_CTX);

	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);

	ercd = snd_dtq(0, DATA0);
	check_ercd(ercd, E_ID);

	ercd = rcv_dtq(0, &data);
	check_ercd(ercd, E_ID);

	ercd = snd_dtq(TNUM_DTQID+1, DATA0);
	check_ercd(ercd, E_ID);

	ercd = rcv_dtq(TNUM_DTQID+1, &data);
	check_ercd(ercd, E_ID);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(2);
	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(4);
	ercd = snd_dtq(DTQ1, DATA1);
	check_ercd(ercd, E_OK);

	check_point(7);
	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_point(11);
	check_assert(data == DATA2);

	ercd = snd_dtq(DTQ1, DATA3);
	check_ercd(ercd, E_OK);

	ercd = ref_dtq(DTQ1, &rdtq);
	check_ercd(ercd, E_OK);

	check_assert(rdtq.stskid == TSK_NONE);

	check_assert(rdtq.rtskid == TSK_NONE);

	check_assert(rdtq.sdtqcnt == 1);

	ercd = snd_dtq(DTQ1, DATA4);
	check_ercd(ercd, E_OK);

	ercd = ref_dtq(DTQ1, &rdtq);
	check_ercd(ercd, E_OK);

	check_assert(rdtq.stskid == TSK_NONE);

	check_assert(rdtq.rtskid == TSK_NONE);

	check_assert(rdtq.sdtqcnt == 2);

	ercd = snd_dtq(DTQ1, DATA5);
	check_ercd(ercd, E_OK);

	check_point(13);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(16);
	ercd = snd_dtq(DTQ2, DATA1);
	check_ercd(ercd, E_OK);

	check_point(18);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(26);
	ercd = snd_dtq(DTQ2, DATA0);
	check_ercd(ercd, E_RLWAI);

	check_point(28);
	ercd = snd_dtq(DTQ2, DATA0);
	check_ercd(ercd, E_DLT);

	check_point(30);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_point(31);
	ercd = snd_dtq(DTQ2, DATA0);
	check_ercd(ercd, E_RASTER);

	check_point(33);
	ercd = snd_dtq(DTQ2, DATA0);
	check_ercd(ercd, E_RASTER);

	check_point(34);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(36);
	ercd = ena_ter();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER_UINT	ercd;
	intptr_t	data;

	check_point(3);
	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_point(5);
	check_assert(data == DATA1);

	check_point(6);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(9);
	ercd = snd_dtq(DTQ1, DATA2);
	check_ercd(ercd, E_OK);

	check_point(10);
	ercd = slp_tsk();
	check_ercd(ercd, E_OK);

	check_point(35);
	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_RLWAI);

	check_point(38);
	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_DLT);

	check_point(40);
	ercd = dis_ter();
	check_ercd(ercd, E_OK);

	check_point(41);
	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_RASTER);

	check_point(43);
	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_RASTER);

	check_point(44);
	ercd = ena_ter();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER_UINT	ercd;
	intptr_t	data;

	check_point(8);
	ercd = wup_tsk(TASK2);
	check_ercd(ercd, E_OK);

	check_point(12);
	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_point(14);
	check_assert(data == DATA3);

	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_assert(data == DATA4);

	ercd = rcv_dtq(DTQ1, &data);
	check_ercd(ercd, E_OK);

	check_assert(data == DATA5);

	check_point(15);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(17);
	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_OK);

	check_point(19);
	check_assert(data == DATA1);

	check_point(20);
	ercd = sta_alm(ALM1, TEST_TIME_CP);
	check_ercd(ercd, E_OK);

	check_point(21);
	ercd = rcv_dtq(DTQ2, &data);
	check_ercd(ercd, E_OK);

	check_point(24);
	check_assert(data == DATA2);

	check_point(25);
	ercd = wup_tsk(TASK1);
	check_ercd(ercd, E_OK);

	check_point(27);
	ercd = rel_wai(TASK1);
	check_ercd(ercd, E_OK);

	check_point(29);
	ercd = ini_dtq(DTQ2);
	check_ercd(ercd, E_OK);

	check_point(32);
	ercd = ras_ter(TASK1);
	check_ercd(ercd, E_OK);

	check_point(37);
	ercd = rel_wai(TASK2);
	check_ercd(ercd, E_OK);

	check_point(39);
	ercd = ini_dtq(DTQ2);
	check_ercd(ercd, E_OK);

	check_point(42);
	ercd = ras_ter(TASK2);
	check_ercd(ercd, E_OK);

	check_finish(45);
	check_point(0);
}
