/*
 *  TOPPERS Software
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 * 
 *  Copyright (C) 2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: test_extsvc1.c 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/* 
 *		拡張サービスコールに関するテスト(1)
 *
 * 【テストの目的】
 *
 *  拡張サービスコール機能を網羅的にテストする．
 *
 * 【テスト実施上の注意】
 *
 *  このテストは，拡張サービスコールの呼び出し時にスタック領域が不足し
 *  てE_NOMEMが返るケースと，ネストレベルが255を超えてE_SYSが返るケース
 *  をテストしている．このテストの際の情報を，以下の2つのメッセージ（数
 *  値はターゲットにより異なる）で出力している．
 *
 *  cal_svc returns E_NOMEM when svclevel = 23
 *  cal_svc returns E_SYS when svclevel = 255
 *
 *  このテストを正しく実施するには，タスクのスタックサイズを適切に設定
 *  する必要がある．適切なスタックサイズはターゲット依存となるため，ター
 *  ゲットによっては，スタックサイズを以下のように調整する必要がある．
 *
 *  上記のE_NOMEMが報告されるべきところでE_SYSが報告されてエラー停止す
 *  る場合には，TASK1のシステムスタックサイズが大きすぎる．そのため，
 *  OVERFLOW_STACK_SIZEをデフォルト（STACK_SIZE）よりも小さい値に定義す
 *  る必要がある．
 *
 *  逆にE_SYSが報告されるべきところでE_NOMEMが報告されてエラー停止する
 *  場合には，TASK2のシステムスタックサイズが小さすぎる．そのため，
 *  NON_OVERFLOW_STACK_SIZEをデフォルト（STACK_SIZE * 20）よりも大きい
 *  値に定義する必要がある．逆に，RAM容量が不足する場合には，小さい値に
 *  設定して動作確認する必要がある．
 *
 * 【テストでカバーする仕様タグ】
 *		［NGKI1243］［NGKI0189］
 *		［NGKI3160］［NGKI3162］［NGKI3165］［NGKI3166］［NGKI3167］
 *		［NGKI3192］［NGKI3193］［NGKI3194］
 *		［NGKI3195］［NGKI3197］［NGKI3198］
 *
 * 【考察】
 *
 * 【テスト項目】
 *
 *	(A) ref_texでの拡張サービスコールのネストレベルの参照［NGKI1243］
 *		(A-1) 対象タスクが拡張サービスコールを呼び出していない時
 *		(A-2) 対象タスクが拡張サービスコールを呼び出しており，ネスト段
 *			  数が1の時
 *		(A-3) 対象タスクが拡張サービスコールを呼び出しており，ネスト段
 *			  数が2の時
 *		※ 対象タスクは他タスクでテストする
 *	(B) 特権モードからも拡張サービスコールを呼び出せる［NGKI3160］
 *		(B-1) システムタスクから拡張サービスコールを呼ぶ
 *		(B-2) 非タスクコンテキストから拡張サービスコールを呼ぶ
 *		(B-3) 拡張サービスコールから拡張サービスコールを呼ぶ
 *	(C) 拡張サービスコールがアクセスの主体となる［NGKI3162］
 *		(C-1) ユーザタスクから呼び出した拡張サービスコールから，カーネ
 *			  ルドメインのみがアクセスできるオブジェクトにアクセスできる
 *	(D) cdmidには，拡張サービスコールを呼び出した処理単位が属するドメイ
 *		ンIDが渡される［NGKI3165］
 *		(D-1) システムタスクから呼び出した場合にはTDOM_KERNEL
 *		(D-2) 非タスクコンテキストから呼び出した場合にはTDOM_KERNEL
 *		(D-3) 拡張サービスコールから呼び出した場合にはTDOM_KERNEL
 *		(D-4) ユーザタスクから呼び出した場合には，それが属する保護ドメイン
 *	(E) par1〜par5には，拡張サービスコールに対するパラメータが渡される
 *		［NGKI3166］
 *	(F) cal_svcのエラー検出
 *		(F-1) fncdが0［NGKI3192］
 *		(F-2) fncdがTMAX_FNCDよりも大きい［NGKI3193］
 *		(F-3) fncdに対する拡張サービスコールが未定義［NGKI3194］
 *		(F-4) スタックの残り領域が不足［NGKI3197］
 *		(F-5) ネストレベルが上限を超える［NGKI3198］
 *	(G) 拡張サービスコールの実行開始直後の状態［NGKI0189］
 *		(G-1) CPUロック状態で呼び出した場合
 *		(G-2) CPUロック解除状態で呼び出した場合
 *		(G-3) 割込み優先度マスクがTMAX_INTPRIの状態で呼び出した場合
 *		(G-4) 割込み優先度マスク全解除状態で呼び出した場合
 *		(G-5) ディスパッチ禁止状態で呼び出した場合
 *		(G-6) ディスパッチ許可状態で呼び出した場合
 *	(H) cal_svcの返値
 *		(H-1) 拡張サービスコールの返値を返す［NGKI3195］
 *	(I) 拡張サービスコールのリターン開始直後の状態［NGKI0189］
 *		(I-1) CPUロック状態でリターンした場合
 *		(I-2) CPUロック解除状態でリターンした場合
 *		(I-3) 割込み優先度マスク全解除でない状態でリターンした場合
 *		(I-4) 割込み優先度マスク全解除状態でリターンした場合
 *		(I-5) ディスパッチ禁止状態でリターンした場合
 *		(I-6) ディスパッチ許可状態でリターンした場合
 *	(J) TMAX_FNCDが拡張サービスコールの機能番号の最大値に定義される
 *		［NGKI3167］
 *
 * 【使用リソース】
 *
 *	TASK1: ユーザタスク．拡張サービスコールを呼び出す
 *	TASK2: システムタスク．拡張サービスコールを呼び出す
 *	TASK3: 対象タスクに対して状態参照するためのタスク
 *	ALM1: 拡張サービスコールを呼び出す
 *	EXTSVC1: 拡張サービスコール
 *	EXTSVC2: 多重で呼び出す拡張サービスコール
 *	EXTSVC3: エラーになるまで多重に呼び出すサービスコール
 *
 * 【テストシーケンス】
 *
 *	== TASK1（優先度：10）==
 *	1:	act_tsk(TASK2) -> E_OACV
 *		act_tsk(TASK3)
 *	== TASK3-1（優先度：9，1回目）==
 *	2:	ref_tsk(TASK1, &rtsk)
 *		assert(rtsk.svclevel == 0)							... (A-1)
 *		ext_tsk()
 *	== TASK1（続き）==
 *	3:	cal_svc(0, 0, 0, 0, 0, 0) -> E_RSFN					... (F-1)
 *		cal_svc(TFN_EXTSVC3+1, 0, 0, 0, 0, 0) -> E_RSFN		... (F-2)(J)
 *		cal_svc(TFN_EXTSVC0, 0, 0, 0, 0, 0) -> E_RSFN		... (F-3)
 *		cal_svc(TFN_EXTSVC1, PAR1_1, PAR1_2, PAR1_3, PAR1_4, PAR1_5) \
 *															... (G-2)(G-4)(G-6)
 *	== EXTSVC1-1（1回目）==
 *	4:	state(false, false, TIPM_ENAALL, false, false, true)
 *		assert(par1 == PAR1_1 && par2 == PAR1_2 && par3 == PAR1_3 \
 *				&& par4 == PAR1_4 && par5 == PAR1_5)		... (E)
 *		assert(cdmid == DOM1)								... (D-4)
 *		act_tsk(TASK3)
 *	== TASK3-2（2回目）==
 *	5:	ref_tsk(TASK1, &rtsk)
 *		assert(rtsk.svclevel == 1)							... (A-2)
 *		ext_tsk()
 *	== EXTSVC1-1（続き）==
 *	6:	cal_svc(TFN_EXTSVC2, PAR2_1, PAR2_2, PAR2_3, PAR2_4, PAR2_5) ... (B-3)
 *	== EXTSVC2-1（1回目）==
 *	7:	state(false, false, TIPM_ENAALL, false, false, true)
 *		assert(par1 == PAR2_1 && par2 == PAR2_2 && par3 == PAR2_3 \
 *				&& par4 == PAR2_4 && par5 == PAR2_5)		... (E)
 *		assert(cdmid == TDOM_KERNEL)						... (D-3)
 *		act_tsk(TASK3)
 *	== TASK3-3（3回目）==
 *	8:	ref_tsk(TASK1, &rtsk)
 *		assert(rtsk.svclevel == 2)							... (A-2)
 *		ext_tsk()
 *	== EXTSVC2-1（続き）==
 *	9:	dis_dsp()
 *		chg_ipm(TMAX_INTPRI)
 *		loc_cpu()
 *		RETURN(E_OK)										... (I-1)(I-3)(I-5)
 *	== EXTSVC1-1（続き）==
 *	10:	state(false, true, TMAX_INTPRI, true, true, true)
 *		cal_svc(TFN_EXTSVC2, 0, 0, 0, 0, 0) -> E_PAR		... (G-1)(G-3)(G-5)
 *	== EXTSVC2-2（2回目）==
 *	11:	state(false, true, TMAX_INTPRI, true, true, true)
 *		unl_cpu()
 *		chg_ipm(TIPM_ENAALL)
 *		RETURN(E_PAR)										... (H-1)(I-2)(I-4)
 *	== EXTSVC1-1（続き）==
 *	12:	state(false, false, TIPM_ENAALL, true, true, true)
 *		ena_dsp()											... (I-6)
 *		act_tsk(TASK2)										... (C-1)
 *		RETURN(E_OK)
 *	== TASK1（続き）==
 *	13:	state(false, false, TIPM_ENAALL, false, false, true)
 *		cal_svc(TFN_EXTSVC3, 0, 0, 0, 0, 0) -> E_NOMEM		... (F-4)
 *	.. ここで EXTSVC3 が実行されるが，この処理は手書きする
 *	== TASK1（続き）==
 *	14:	ext_tsk()
 *	== TASK2（優先度：10）==
 *	15:	act_tsk(TASK3)
 *	== TASK3-4（4回目）==
 *	16:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.svclevel == 0)							... (A-1)
 *		ext_tsk()
 *	== TASK2（続き）==
 *	17:	cal_svc(0, 0, 0, 0, 0, 0) -> E_RSFN					... (F-1)
 *		cal_svc(TFN_EXTSVC3+1, 0, 0, 0, 0, 0) -> E_RSFN		... (F-2)
 *		cal_svc(TFN_EXTSVC0, 0, 0, 0, 0, 0) -> E_RSFN		... (F-3)
 *		cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)					... (B-1)
 *	== EXTSVC1-2（2回目）==
 *	18:	state(false, false, TIPM_ENAALL, false, false, true)
 *		assert(cdmid == TDOM_KERNEL)						... (D-1)
 *		act_tsk(TASK3)
 *	== TASK3-5（5回目）==
 *	19:	ref_tsk(TASK2, &rtsk)
 *		assert(rtsk.svclevel == 1)							... (A-2)
 *		ext_tsk()
 *	== EXTSVC1-2（続き）==
 *	20:	RETURN(E_OK)
 *	== TASK2（続き）==
 *	21:	cal_svc(TFN_EXTSVC3, 0, 0, 0, 0, 0) -> E_SYS		... (F-4)
 *	.. ここで EXTSVC3 が実行されるが，この処理は手書きする
 *	== TASK2（続き）==
 *	22:	sta_alm(ALM1, 0U)
 *		dly_tsk(1U)
 *	== ALM1 ==
 *	23:	cal_svc(0, 0, 0, 0, 0, 0) -> E_RSFN					... (F-1)
 *		cal_svc(TFN_EXTSVC3+1, 0, 0, 0, 0, 0) -> E_RSFN		... (F-2)
 *		cal_svc(TFN_EXTSVC0, 0, 0, 0, 0, 0) -> E_RSFN		... (F-3)
 *		cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0)					... (B-2)
 *	== EXTSVC1-3（3回目）==
 *	24:	state_i(true, false, false, true, true)
 *		assert(cdmid == TDOM_KERNEL)						... (D-2)
 *		RETURN(E_OK)
 *	== ALM1（続き）==
 *	25:	state_i(true, false, false, true, true)
 *		RETURN
 *	== TASK2（続き）==
 *	26:	END
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "kernel_cfg.h"
#include "test_extsvc1.h"

#define PAR1_1		((intptr_t) 1)
#define PAR1_2		((intptr_t) 2)
#define PAR1_3		((intptr_t) 3)
#define PAR1_4		((intptr_t) 4)
#define PAR1_5		((intptr_t) 5)

#define PAR2_1		((intptr_t) -5)
#define PAR2_2		((intptr_t) -4)
#define PAR2_3		((intptr_t) -3)
#define PAR2_4		((intptr_t) -2)
#define PAR2_5		((intptr_t) -1)

/*
 *  extsvc3_routineは生成されない
 *
 *  この拡張サービスコールでは，ネストレベルの最大をテストするため，こ
 *  の中から拡張サービスコールを呼んでも，正しく呼べるとは限らない．
 */

ER_UINT
extsvc3_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	ER		ercd, ercd1;
	T_RTSK	rtsk;

	ercd = cal_svc(TFN_EXTSVC3, 0, 0, 0, 0, 0);
	if (ercd < 0) {
		ercd1 = ref_tsk(TSK_SELF, &rtsk);
		check_ercd(ercd1, E_OK);

		if (SERCD(ercd) == -1) {
			/*
			 *  最もネストの深い拡張サービスコールで1度だけメッセージを
			 *  出力する．
			 *
			 *  cal_svcがE_SYSを返す状況では，拡張サービスコール呼出し
			 *  を含むsyslog関数は使えないため，syslog_wri_logを関数呼
			 *  出しする．
			 */
			{
				SYSLOG	logbuf;
				char	*msg = "cal_svc returns %s when svclevel = %d";

				logbuf.logtype = LOG_TYPE_COMMENT;
				logbuf.loginfo[0] = (intptr_t) msg;
				logbuf.loginfo[1] = (intptr_t) itron_strerror(ercd);
				logbuf.loginfo[2] = (intptr_t) rtsk.svclevel;
				(void) _syslog_syslog_wri_log(LOG_NOTICE, &logbuf);
			}
			ercd = ERCD(MERCD(ercd), -2);
		}
		if (rtsk.svclevel == 1) {
			return(ERCD(MERCD(ercd), -1));
		}
	}
	return(ercd);
}

/*
 *  ここから下が生成される
 */

void
alarm1_handler(intptr_t exinf)
{
	ER		ercd;

	check_point(23);

	ercd = cal_svc(0, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC3+1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC0, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(25);
	check_state_i(true, false, false, true, true);

	return;

	check_point(0);
}

static uint_t	extsvc1_count = 0;

ER_UINT
extsvc1_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	ER		ercd;

	switch (++extsvc1_count) {
	case 1:
		check_point(4);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		check_assert(par1 == PAR1_1 && par2 == PAR1_2 && par3 == PAR1_3 && par4 == PAR1_4 && par5 == PAR1_5);
		check_assert(cdmid == DOM1);

		ercd = act_tsk(TASK3);
		check_ercd(ercd, E_OK);

		check_point(6);

		ercd = cal_svc(TFN_EXTSVC2, PAR2_1, PAR2_2, PAR2_3, PAR2_4, PAR2_5);
		check_ercd(ercd, E_OK);

		check_point(10);
		check_state(false, true, TMAX_INTPRI, true, true, true);

		ercd = cal_svc(TFN_EXTSVC2, 0, 0, 0, 0, 0);
		check_ercd(ercd, E_PAR);

		check_point(12);
		check_state(false, false, TIPM_ENAALL, true, true, true);

		ercd = ena_dsp();
		check_ercd(ercd, E_OK);

		ercd = act_tsk(TASK2);
		check_ercd(ercd, E_OK);

		return(E_OK);

		check_point(0);

	case 2:
		check_point(18);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		check_assert(cdmid == TDOM_KERNEL);

		ercd = act_tsk(TASK3);
		check_ercd(ercd, E_OK);

		check_point(20);

		return(E_OK);

		check_point(0);

	case 3:
		check_point(24);
		check_state_i(true, false, false, true, true);
		check_assert(cdmid == TDOM_KERNEL);

		return(E_OK);

		check_point(0);
	}
	check_point(0);
	return(E_SYS);
}

static uint_t	extsvc2_count = 0;

ER_UINT
extsvc2_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	ER		ercd;

	switch (++extsvc2_count) {
	case 1:
		check_point(7);
		check_state(false, false, TIPM_ENAALL, false, false, true);
		check_assert(par1 == PAR2_1 && par2 == PAR2_2 && par3 == PAR2_3 && par4 == PAR2_4 && par5 == PAR2_5);
		check_assert(cdmid == TDOM_KERNEL);

		ercd = act_tsk(TASK3);
		check_ercd(ercd, E_OK);

		check_point(9);

		ercd = dis_dsp();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TMAX_INTPRI);
		check_ercd(ercd, E_OK);

		ercd = loc_cpu();
		check_ercd(ercd, E_OK);

		return(E_OK);

		check_point(0);

	case 2:
		check_point(11);
		check_state(false, true, TMAX_INTPRI, true, true, true);

		ercd = unl_cpu();
		check_ercd(ercd, E_OK);

		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);

		return(E_PAR);

		check_point(0);
	}
	check_point(0);
	return(E_SYS);
}

void
task1(intptr_t exinf)
{
	ER		ercd;

	check_point(1);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OACV);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(3);

	ercd = cal_svc(0, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC3+1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC0, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC1, PAR1_1, PAR1_2, PAR1_3, PAR1_4, PAR1_5);
	check_ercd(ercd, E_OK);

	check_point(13);
	check_state(false, false, TIPM_ENAALL, false, false, true);

	ercd = cal_svc(TFN_EXTSVC3, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_NOMEM);

	check_point(14);

	ercd = ext_tsk();
	check_ercd(ercd, E_OK);

	check_point(0);
}

void
task2(intptr_t exinf)
{
	ER		ercd;

	check_point(15);

	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	check_point(17);

	ercd = cal_svc(0, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC3+1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC0, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_RSFN);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	check_point(21);

	ercd = cal_svc(TFN_EXTSVC3, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_SYS);

	check_point(22);

	ercd = sta_alm(ALM1, 0U);
	check_ercd(ercd, E_OK);

	ercd = dly_tsk(1U);
	check_ercd(ercd, E_OK);

	check_finish(26);

	check_point(0);
}

static uint_t	task3_count = 0;

void
task3(intptr_t exinf)
{
	ER		ercd;
	T_RTSK	rtsk;

	switch (++task3_count) {
	case 1:
		check_point(2);

		ercd = ref_tsk(TASK1, &rtsk);
		check_ercd(ercd, E_OK);
		check_assert(rtsk.svclevel == 0);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 2:
		check_point(5);

		ercd = ref_tsk(TASK1, &rtsk);
		check_ercd(ercd, E_OK);
		check_assert(rtsk.svclevel == 1);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 3:
		check_point(8);

		ercd = ref_tsk(TASK1, &rtsk);
		check_ercd(ercd, E_OK);
		check_assert(rtsk.svclevel == 2);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 4:
		check_point(16);

		ercd = ref_tsk(TASK2, &rtsk);
		check_ercd(ercd, E_OK);
		check_assert(rtsk.svclevel == 0);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);

	case 5:
		check_point(19);

		ercd = ref_tsk(TASK2, &rtsk);
		check_ercd(ercd, E_OK);
		check_assert(rtsk.svclevel == 1);

		ercd = ext_tsk();
		check_ercd(ercd, E_OK);

		check_point(0);
	}
	check_point(0);
}
