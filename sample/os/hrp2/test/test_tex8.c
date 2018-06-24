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
 *  $Id: test_tex8.c 837 2012-12-26 15:09:59Z ertl-hiro $
 */

/* 
 *		タスク例外処理に関するテスト(8)
 *
 * 【テストの目的】
 *
 *  タスク例外処理ルーチンが連続して実行開始された場合に，タスクのスタッ
 *  ク領域の使用量が増加しないことを確認する．また，タスク例外処理ルー
 *  チンからのリターン時に，ディスパッチャを呼ぶ前にタスクのenatexを
 *  falseにしている処理が正しく実装されているかも確認する．
 *
 *  カーネルドメインに属するタスクと，ユーザドメインに属するタスクの両
 *  者に対して，同等のテストを実施する．
 *
 * 【考察】
 *
 *  タスク例外処理ルーチンからのリターン時に，ディスパッチャを呼ぶ前に
 *  タスクのenatexをfalseにしている処理が正しく実装されているかは，次の
 *  方法で確認する．
 *
 *  タスク例外処理ルーチンの中で，ディスパッチ禁止状態かつタスク例外処
 *  理許可状態とした後，高優先度タスクを起動する．この時点では，高優先
 *  度タスクへは切り換わらない．タスク例外処理ルーチンからリターンする
 *  と，ディスパッチ禁止が解除され，高優先度タスクへ切り換わる．高優先
 *  度タスクは，対象タスクに対してタスク例外処理を要求した後，終了する
 *  (*)．enatexをfalseにする処理が正しく実装されていないと，高優先度タ
 *  スクから対象タスクに切り換えるディスパッチャにより，タスク例外処理
 *  ルーチンが実行開始され，スタック領域の使用量が増加する．それに対し
 *  て，enatexをfalseにする処理が正しく実装されていると，スタック領域の
 *  使用量は増加しない．
 *
 *  そこで，上記の(*)の状況を繰り返し起こしても，スタック領域の不足が生
 *  じないことを確認する．
 *
 * 【テスト項目】
 *
 *	(A) カーネルドメインに属するタスクが，タスク例外処理ルーチンを連続
 *		して実行開始する状況を繰り返した場合に，スタック領域の不足が生
 *		じない
 *  (B) カーネルドメインに属するタスクに対して，(*)の状況を繰り返した場
 *		合に，スタック領域の不足が生じない
 *	(C) ユーザドメインに属するタスクが，タスク例外処理ルーチンを連続し
 *		て実行開始する状況を繰り返した場合に，スタック領域の不足が生じ
 *		ない
 *  (D) ユーザドメインに属するタスクに対して，(*)の状況を繰り返した場合
 *		に，スタック領域の不足が生じない
 *
 * 【使用リソース】
 *
 *	TASK1: カーネルドメインに属するテスト対象のタスク
 *	TASK2: TASK1にタスク例外処理を要求する高優先度タスク
 *	TASK3: ユーザドメインに属するテスト対象のタスク
 *	TASK4: TASK3にタスク例外処理を要求する高優先度タスク
 *
 * 【テストシーケンス】
 *
 *  このテストプログラムをgentestによって生成することはあきらめる．
 */

#include <kernel.h>
#include <test_lib.h>
#include <t_syslog.h>
#include "kernel_cfg.h"
#include "test_tex8.h"

#define STASK_TEX_COUNT		((uint_t) STACK_SIZE)
#define UTASK_TEX_COUNT		((uint_t) STACK_SIZE)
//#define UTASK_TEX_COUNT		200

static uint_t	tex_start_count;

void
task1(intptr_t exinf)
{
	ER		ercd;

	check_point(1);

	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	/*
	 *  テスト項目(A)のテスト
	 */
	tex_start_count = 0U;
	ercd = ras_tex(TASK1, 0x0001);
	check_ercd(ercd, E_OK);
	check_assert(tex_start_count == STASK_TEX_COUNT);

	check_point(2);

	/*
	 *  テスト項目(B)のテスト
	 */
	tex_start_count = 0U;
	ercd = ras_tex(TASK1, 0x0002);
	check_ercd(ercd, E_OK);
	check_assert(tex_start_count == STASK_TEX_COUNT);

	check_point(3);

	/*
	 *  この後のテストはTASK3で実施
	 */
	ercd = act_tsk(TASK3);
	check_ercd(ercd, E_OK);

	ercd = ext_tsk();
	check_point(0);
}

void
tex_task1(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	/*
	 *  スタック領域に余裕があることを，拡張サービスコールを使って確認
	 *  する．
	 */
	ercd = cal_svc(TFN_EXTSVC, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

#ifdef DEBUG_LOG
	if (tex_start_count % 50 == 0) {
		syslog_2(LOG_NOTICE, "%d: sp = %x", tex_start_count, &ercd);
	}
#endif /* DEBUG_LOG */

	tex_start_count += 1;
	if (tex_start_count < STASK_TEX_COUNT) {
		switch (texptn) {
		case 0x0001:
			ercd = ras_tex(TASK1, 0x0001);
			check_ercd(ercd, E_OK);
			break;

		case 0x0002:
			ercd = dis_dsp();
			check_ercd(ercd, E_OK);

			ercd = ena_tex();
			check_ercd(ercd, E_OK);

			ercd = act_tsk(TASK2);
			check_ercd(ercd, E_OK);
			break;
		}
	}
}

void
task2(intptr_t exinf)
{
	ER		ercd;

	ercd = ras_tex(TASK1, 0x0002);
	check_ercd(ercd, E_OK);
	
	ercd = ext_tsk();
	check_point(0);
}

void
task3(intptr_t exinf)
{
	ER		ercd;

	check_point(4);

	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	/*
	 *  テスト項目(C)のテスト
	 */
	tex_start_count = 0U;
	ercd = ras_tex(TASK3, 0x0001);
	check_ercd(ercd, E_OK);
	check_assert(tex_start_count == UTASK_TEX_COUNT);

	check_point(5);

	/*
	 *  テスト項目(D)のテスト
	 */
	tex_start_count = 0U;
	ercd = ras_tex(TASK3, 0x0002);
	check_ercd(ercd, E_OK);
	check_assert(tex_start_count == UTASK_TEX_COUNT);

	check_finish(6);
}

void
tex_task3(TEXPTN texptn, intptr_t exinf)
{
	ER		ercd;

	/*
	 *  システムスタック領域に余裕があることを，拡張サービスコールを使っ
	 *  て確認する．
	 *
	 *  ユーザタスクにおいて，システムスタック領域が減ることはないはず
	 *  だが，カーネルのバグにより増減する可能性はあるため，念のため確
	 *  認している．ただし，システムスタック領域が増えてしまうバグは，
	 *  これでは検出できない（不可解な動作をする）．
	 */
	ercd = cal_svc(TFN_EXTSVC, 0, 0, 0, 0, 0);
	check_ercd(ercd, E_OK);

#ifdef DEBUG_LOG
	if (tex_start_count % 50 == 0) {
		syslog_2(LOG_NOTICE, "%d: sp = %x", tex_start_count, &ercd);
	}
#endif /* DEBUG_LOG */

	tex_start_count += 1;
	if (tex_start_count < UTASK_TEX_COUNT) {
		switch (texptn) {
		case 0x0001:
			ercd = ras_tex(TASK3, 0x0001);
			check_ercd(ercd, E_OK);
			break;

		case 0x0002:
			ercd = dis_dsp();
			check_ercd(ercd, E_OK);

			ercd = ena_tex();
			check_ercd(ercd, E_OK);

			ercd = act_tsk(TASK4);
			check_ercd(ercd, E_OK);
			break;
		}
	}
}

void
task4(intptr_t exinf)
{
	ER		ercd;

	ercd = ras_tex(TASK3, 0x0002);
	check_ercd(ercd, E_OK);
	
	ercd = ext_tsk();
	check_point(0);
}

ER_UINT
extsvc_routine(intptr_t par1, intptr_t par2, intptr_t par3,
								intptr_t par4, intptr_t par5, ID cdmid)
{
	return(E_OK);
}
