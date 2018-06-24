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
 *  $Id: test_diswai1.c 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/* 
 *		待ち禁止状態に関するテスト(1)
 *
 * 【テストの目的】
 *
 *  カーネルドメインのタスクをdis_wai／idis_waiする処理を網羅的にテスト
 *  する．
 *
 * 【テストでカバーする仕様タグ】
 *		［NGKI1321］［NGKI1322］
 *		［NGKI1330］［NGKI1331］［NGKI1332］
 *		［NGKI1333］［NGKI1334］
 *
 * 【考察】
 *
 *  システムタスクに対しては，タスク例外処理マスク状態は，拡張サービス
 *  コールを実行している間と定義されている．ここで，「拡張サービスコー
 *  ルを実行している間」には，実行可能状態や広義の待ち状態になっている
 *  場合を含んでいる．
 *
 *  そこで，タスク例外処理マスク状態である場合のテストは，次の5つの状態
 *  で実施する．
 *  (a) タスクが拡張サービスコールを実行している間
 *  (b) タスクが拡張サービスコールを実行している間にプリエンプトされ，
 *		実行可能状態になっている状態
 *  ※「実行可能状態や広義の待ち状態になっている場合」は，タスクがプリ
 *     エンプトされて，実行可能状態になっている場合のみでテストする．
 *
 *  一方，タスク例外処理マスク状態でない場合のテストは，次の2つの状態で
 *  実施する．
 *	(c) タスクが実行開始前
 *	(d) タスクが実行開始後で，拡張サービスコールを呼んでいない状態
 *
 * 【テスト項目】
 *
 *	(A) 他タスクに対するdis_waiのエラー検出
 *		(A-1) 対象タスクが休止状態［NGKI1330］
 *		↓対象タスクがタスク例外処理マスク状態でない［NGKI1331］
 *		(A-2) 対象タスクが実行開始前［NGKI1331(c)］
 *		(A-3) 対象タスクが実行開始後で，拡張サービスコールを呼んでいな
 *			  い状態［NGKI1331(d)］
 *		(A-4) 対象タスクが待ち禁止状態［NGKI1332］
 *	(B) 他タスクに対するdis_waiの正常処理［NGKI1333］
 *		(B-1) 対象タスクが拡張サービスコールを実行している間にプリエン
 *			  プトされ，実行状態以外の状態になっている状態［NGKI1333(b)］
 *		※ シングルプロセッサでは，(a)の状態はない
 *	(C) 自タスクに対するdis_waiのエラー検出
 *		※ 自タスクは休止状態になることはない
 *		※ 自タスクに対しては，(c)の状態はない
 *		(C-1) 自タスク拡張サービスコールを呼んでいない状態［NGKI1331(d)］
 *		(C-2) 自タスクが待ち禁止状態［NGKI1332］
 *	(D) 自タスクに対するdis_waiの正常処理［NGKI1333］
 *		(D-1) 自タスクが拡張サービスコールを実行している間［NGKI1333(a)］
 *		※ 自タスクに対するdis_waiでは，(b)の状態はない
 *		(D-2) tskidにTSK_SELFを指定した場合［NGKI1334］
 *	(E) idis_waiのエラー検出
 *		(E-1) 対象タスクが休止状態［NGKI1330］
 *		↓対象タスクがタスク例外処理マスク状態でない［NGKI1331］
 *		(E-2) 対象タスクが実行開始前［NGKI1331(c)］
 *		(E-3) 対象タスクが実行開始後で，拡張サービスコールを呼んでいな
 *			  い状態［NGKI1331(d)］
 *		(E-4) 対象タスクが待ち禁止状態［NGKI1332］
 *	(F) idis_waiの正常処理［NGKI1333］
 *		(F-1) 対象タスクが拡張サービスコールを実行している間［NGKI1333(a)］
 *		(F-2) 対象タスクが拡張サービスコールを実行している間にプリエン
 *			  プトされ，実行状態以外の状態になっている状態［NGKI1333(b)］
 *	(G) 拡張サービスコールの多重呼出し
 *		(G-1) 多重の内側の拡張サービスコールにいるタスクに対して
 *			  dis_waiを発行
 *		(G-2) 多重の外側の拡張サービスコールにいるタスクに対して
 *			  dis_waiを発行
 *
 * 【使用リソース】
 *
 *	TASK1: メインタスク．自タスクに対してタスク例外処理を要求する
 *	TASK2: メインタスクに対してタスク例外処理を要求するタスク
 *	TASK3: 実行開始前のタスク
 *	TASK4: 休止状態のタスク
 *	ALM1: メインタスクに対してタスク例外処理を要求する割込み処理
 *	EXTSVC1: 拡張サービスコール
 *	EXTSVC2: 多重で呼び出す拡張サービスコール
 *
 * 【テストフェーズ】
 *
 *	フェーズ1：簡単にテストできる項目をテスト
 *				(C-1),(A-1),(A-2),(E-1),(E-2),(E-3)
 *	フェーズ2：(A-3)をテスト
 *	フェーズ3：(B-1)をテスト，(A-4)(C-2)(D-2)もテストする
 *	フェーズ4：(D-1)をテスト
 *	フェーズ5：(F-1)をテスト，(E-4)もテストする
 *	フェーズ6：(F-2)をテスト
 *	フェーズ7：(G-1)をテスト
 *	フェーズ8：(G-2)をテスト
 *
 * 【テストシーケンス】
 *
 *  このテストプログラムをgentestによって生成することはあきらめる．
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_diswai1.h"

volatile uint_t	test_phase;
volatile uint_t	flag1, loop_count;
volatile bool_t	flag2;
volatile bool_t	flag3;

/*
 *	フェーズ1：簡単にテストできる項目をテスト
 *		(C-1)：TASK1から，TASK1にdis_waiを発行
 *		(A-1),(A-2)：TASK1から，TASK4とTASK3にdis_waiを発行
 *		(E-1),(E-2),(E-3)：割込み処理（ALM1）から，TASK4，TASK3，TASK1に
 *		idis_waiを発行
 */
void
task1_phase1(void)
{
	T_RTSK	rtsk;
	ER		ercd;

	check_point(1);

	ercd = dis_wai(TASK1);		/* (C-1) */
	check_ercd(ercd, E_OBJ);

	ercd = ref_tsk(TASK1, &rtsk);	/* 待ち禁止状態でないことを確認(1) */
	check_ercd(ercd, E_OK);
	check_assert(rtsk.waifbd == false);

	ercd = dly_tsk(0);				/* 待ち禁止状態でないことを確認(2) */
	check_ercd(ercd, E_OK);

	check_point(2);

	ercd = dis_wai(TASK4);		/* (A-1) */
	check_ercd(ercd, E_OBJ);

	ercd = dis_wai(TASK3);		/* (A-2) */
	check_ercd(ercd, E_OBJ);

	check_point(3);

	flag2 = true;
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);
	while (flag2) ;
}

void
alarm1_phase1(void)
{
	ER		ercd;

	check_point(4);

	ercd = idis_wai(TASK4);		/* (E-1) */
	check_ercd(ercd, E_OBJ);

	ercd = idis_wai(TASK3);		/* (E-2) */
	check_ercd(ercd, E_OBJ);

	ercd = idis_wai(TASK1);		/* (E-3) */
	check_ercd(ercd, E_OBJ);

	flag2 = false;
}

/*
 *	フェーズ2：(A-3)をテスト
 *		TASK1からTASK2を起動し，TASK2からTASK1にdis_waiを発行する．
 */
void
task1_phase2(void)
{
	ER		ercd;

	check_point(5);

	ercd = act_tsk(TASK2);
	check_ercd(ercd, E_OK);
}

void
task2_phase2(void)
{
	T_RTSK	rtsk;
	ER		ercd;

	ercd = ref_tsk(TASK1, &rtsk);
	check_ercd(ercd, E_OK);
	check_assert(rtsk.texmsk == false);

	ercd = dis_wai(TASK1);		/* (A-3) */
	check_ercd(ercd, E_OBJ);

	ercd = ref_tsk(TASK1, &rtsk);
	check_ercd(ercd, E_OK);
	check_assert(rtsk.waifbd == false);

	ercd = ras_tex(TASK1, 0x0001U);
	check_ercd(ercd, E_OK);
}

/*
 *	フェーズ3：(B-1)をテスト，(C-2)(D-2)もテストする
 *		TASK1からEXTSVC1を呼び出し，その間に発生した割込み処理（ALM1）
 *		からTASK2を起動する．TASK2からTASK1にdis_waiを発行する．さらに，
 *		EXTSVC1からTASK1とTSK_SELFに対してdis_waiを発行して，(C-2)と
 *		(D-2)のテストも実施する．
 *		また，TASK2からTASK1にタスク例外処理を要求し，タスク例外処理ルー
 *		チンがEXSVC1からのリターン後に実行されることを確認する．
 */
void
task1_phase3(void)
{
	ER		ercd;

	check_point(6);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);
}

void
tex_task1_phase3(void)
{
	check_point(11);
}

void
extsvc1_phase3(void)
{
	T_RTSK	rtsk;
	ER		ercd;

	check_point(7);

	flag2 = true;
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);
	while (flag2) ;				/* この間に(B-1)をテスト */

	ercd = ref_tsk(TASK1, &rtsk);	/* 待ち禁止状態であることを確認(1) */
	check_ercd(ercd, E_OK);
	check_assert(rtsk.waifbd == true);

	ercd = dly_tsk(1);				/* 待ち禁止状態であることを確認(2) */
	check_ercd(ercd, E_RLWAI);

	check_point(9);

	ercd = dis_wai(TASK1);		/* (C-2) */
	check_ercd(ercd, E_QOVR);

	ercd = dis_wai(TSK_SELF);	/* (D-2) */
	check_ercd(ercd, E_QOVR);

	check_point(10);
}

void
alarm1_phase3(void)
{
	ER		ercd;

	ercd = iact_tsk(TASK2);
	check_ercd(ercd, E_OK);
}

void
task2_phase3(void)
{
	ER		ercd;

	check_point(8);

	ercd = dis_wai(TASK1);		/* (B-1) */
	check_ercd(ercd, E_OK);

	ercd = dis_wai(TASK1);		/* (A-4) */
	check_ercd(ercd, E_QOVR);

	ercd = ras_tex(TASK1, 0x0002U);
	check_ercd(ercd, E_OK);

	flag2 = false;
}

/*
 *	フェーズ4：(D-1)をテスト
 *		TASK1からEXTSVC1を呼び出し，そこからTASK1にdis_waiを発行する．
 */
void
task1_phase4(void)
{
	ER		ercd;

	check_point(12);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);
}

void
extsvc1_phase4()
{
	T_RTSK	rtsk;
	ER		ercd;

	check_point(13);

	ercd = dis_wai(TASK1);		/* (D-1) */
	check_ercd(ercd, E_OK);

	ercd = ref_tsk(TASK1, &rtsk);	/* 待ち禁止状態であることを確認(1) */
	check_ercd(ercd, E_OK);
	check_assert(rtsk.waifbd == true);

	ercd = dly_tsk(1);				/* 待ち禁止状態であることを確認(2) */
	check_ercd(ercd, E_RLWAI);

	check_point(14);
}

/*
 *	フェーズ5：(F-1)をテスト，(E-4)もテストする
 *		TASK1からEXTSVC1を呼び出し，EXTSVC1の実行中に発生した割込み処理
 *		（ALM1）からTASK1にidis_waiを発行する．さらに，TASK1にidis_wai
 *		を発行して，(E-4) のテストも実施する．
 */
void
task1_phase5(void)
{
	ER		ercd;

	check_point(15);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);
}

void
extsvc1_phase5()
{
	ER		ercd;

	check_point(16);

	flag2 = true;
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);
	while (flag2) ;
}

void
alarm1_phase5(void)
{
	ER		ercd;

	check_point(17);

	ercd = idis_wai(TASK1);		/* (F-1) */
	check_ercd(ercd, E_OK);

	ercd = idis_wai(TASK1);		/* (E-4) */
	check_ercd(ercd, E_QOVR);

	flag2 = false;
}

/*
 *	フェーズ6：(F-2)をテスト
 *		TASK1からEXTSVC1を呼び出し，その間に発生した割込み処理（ALM1）
 *		からTASK2を起動する．TASK2の実行中に発生した割込み処理（ALM2）
 *		からTASK1にidis_waiを発行する．
 */
void
task1_phase6(void)
{
	ER		ercd;

	check_point(18);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);
}

void
extsvc1_phase6()
{
	ER		ercd;

	check_point(19);

	flag2 = true;
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);
	while (flag2) ;
}

void
alarm1_phase6(void)
{
	ER		ercd;

	check_point(20);

	ercd = iact_tsk(TASK2);
	check_ercd(ercd, E_OK);

	flag2 = false;
}

void
task2_phase6(void)
{
	ER		ercd;

	check_point(21);

	flag3 = true;
	ercd = sta_alm(ALM2, 0);
	check_ercd(ercd, E_OK);
	while (flag3) ;
}

void
alarm2_phase6(void)
{
	ER		ercd;

	check_point(22);

	ercd = idis_wai(TASK1);			/* (F-2) */
	check_ercd(ercd, E_OK);

	flag3 = false;
}

/*
 *	フェーズ7：(G-1)をテスト
 *		TASK1からEXTSVC1を呼び出し，さらにそこからEXTSVC2を呼び出し，
 *		EXTSVC2を実行中に発生した割込み処理（ALM1）からTASK1にidis_wai
 *		を発行する．
 *		さらに，EXTSVC1に戻った後に，EXTSVC1からTASK1にdis_waiを行い，
 *		エラーになることを確認する．
 *		また，ALM1からTASK1にタスク例外処理を要求し，タスク例外処理ルー
 *		チンがEXSVC1からのリターン後に実行されることを確認する．
 */
void
task1_phase7(void)
{
	ER		ercd;

	check_point(23);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);
}

void
tex_task1_phase7(void)
{
	check_point(29);
}

void
extsvc1_phase7(void)
{
	ER		ercd;

	check_point(24);

	ercd = cal_svc(TFN_EXTSVC2, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	ercd = dis_wai(TASK1);
	check_ercd(ercd, E_QOVR);

	check_point(28);
}

void
extsvc2_phase7(void)
{
	ER		ercd;

	check_point(25);

	flag2 = true;
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);
	while (flag2) ;

	check_point(27);
}

void
alarm1_phase7(void)
{
	ER		ercd;

	check_point(26);

	ercd = idis_wai(TASK1);
	check_ercd(ercd, E_OK);

	ercd = iras_tex(TASK1, 0x0002U);
	check_ercd(ercd, E_OK);

	flag2 = false;
}

/*
 *	フェーズ8：(G-2)をテスト
 *		TASK1からEXTSVC1を呼び出し，そこからTASK1にdis_waiを発行する．
 *		続いてEXTSVC1からEXTSVC2を呼び出し，EXTSVC2を実行中に発生した割
 *		込み処理（ALM1）からTASK1にidis_waiを発行する（エラーになる）．
 *		さらに，EXTSVC1に戻った後に，EXTSVC1からTASK1にdis_waiを行い，
 *		エラーになることを確認する．
 *		また，ALM1からTASK1にタスク例外処理を要求し，タスク例外処理ルー
 *		チンがEXSVC1からのリターン後に実行されることを確認する．
 */
void
task1_phase8(void)
{
	ER		ercd;

	check_point(30);

	ercd = cal_svc(TFN_EXTSVC1, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);
}

void
tex_task1_phase8(void)
{
	check_point(36);
}

void
extsvc1_phase8(void)
{
	ER		ercd;

	check_point(31);

	ercd = dis_wai(TASK1);
	check_ercd(ercd, E_OK);

	ercd = cal_svc(TFN_EXTSVC2, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

	ercd = dis_wai(TASK1);
	check_ercd(ercd, E_QOVR);

	check_point(35);
}

void
extsvc2_phase8(void)
{
	ER		ercd;

	check_point(32);

	flag2 = true;
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);
	while (flag2) ;

	check_point(34);
}

void
alarm1_phase8(void)
{
	ER		ercd;

	check_point(33);

	ercd = idis_wai(TASK1);
	check_ercd(ercd, E_QOVR);

	ercd = iras_tex(TASK1, 0x0002U);
	check_ercd(ercd, E_OK);

	flag2 = false;
}

/*
 *  各処理単位の分岐処理
 */
void
task1(intptr_t exinf)
{
	ER		ercd;

	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	test_phase = 1U;
	task1_phase1();

	test_phase = 2U;
	task1_phase2();

	test_phase = 3U;
	task1_phase3();

	test_phase = 4U;
	task1_phase4();

	test_phase = 5U;
	task1_phase5();

	test_phase = 6U;
	task1_phase6();

	test_phase = 7U;
	task1_phase7();

	test_phase = 8U;
	task1_phase8();

	check_finish(37);
}

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	if ((texptn & 0x0001U) != 0) {
		flag2 = false;
	}
	if ((texptn & 0x0002U) != 0) {
		switch (test_phase) {
		case 3U:
			tex_task1_phase3();
			break;
		case 7U:
			tex_task1_phase7();
			break;
		case 8U:
			tex_task1_phase8();
			break;
		}
	}
}

void
task2(intptr_t exinf)
{
	switch (test_phase) {
	case 2U:
		task2_phase2();
		break;
	case 3U:
		task2_phase3();
		break;
	case 6U:
		task2_phase6();
		break;
	default:
		check_point(0);
		break;
	}
}

void
task3(intptr_t exinf)
{
}

void
task4(intptr_t exinf)
{
}

void
alarm1_handler(intptr_t exinf)
{
	switch (test_phase) {
	case 1U:
		alarm1_phase1();
		break;
	case 3U:
		alarm1_phase3();
		break;
	case 5U:
		alarm1_phase5();
		break;
	case 6U:
		alarm1_phase6();
		break;
	case 7U:
		alarm1_phase7();
		break;
	case 8U:
		alarm1_phase8();
		break;
	default:
		check_point(0);
		break;
	}
}

void
alarm2_handler(intptr_t exinf)
{
	switch (test_phase) {
	case 6U:
		alarm2_phase6();
		break;
	default:
		check_point(0);
		break;
	}
}

ER_UINT
extsvc1_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	switch (test_phase) {
	case 3U:
		extsvc1_phase3();
		break;
	case 4U:
		extsvc1_phase4();
		break;
	case 5U:
		extsvc1_phase5();
		break;
	case 6U:
		extsvc1_phase6();
		break;
	case 7U:
		extsvc1_phase7();
		break;
	case 8U:
		extsvc1_phase8();
		break;
	default:
		check_point(0);
		break;
	}
	return(E_OK);
}

ER_UINT
extsvc2_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	switch (test_phase) {
	case 7U:
		extsvc2_phase7();
		break;
	case 8U:
		extsvc2_phase8();
		break;
	default:
		check_point(0);
		break;
	}
	return(E_OK);
}
