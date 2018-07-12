/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2012 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2012 by FUJISOFT INCORPORATED
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
 *  $Id: out.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */
#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "ttsp_test_lib.h"
#include "out.h"

/* 割込みが発生したか判別するためのフラグ */
bool_t int_flag;

void main_task(intptr_t exinf) {
	SIL_PRE_LOC;
	ER ercd;

	/* 初期化ルーチン内で発生させた割込みを待つ */
	wait_raise_int();

	ttsp_check_point(1);

	/* 
	 * タスク
	 */
	syslog_0(LOG_NOTICE, "=== test start from TASK ===");
	all_test();
	ttsp_check_point(2);


	/* 
	 * タスク例外処理ルーチン
	 */
	syslog_0(LOG_NOTICE, "=== test start from TASK EXPECT ===");
	ercd = ena_tex();
	check_ercd(ercd, E_OK);
	ercd = ras_tex(TSK_SELF, 0x00000001);
	check_ercd(ercd, E_OK);
	ttsp_check_point(4);
	ercd = dis_tex();
	check_ercd(ercd, E_OK);


	/* 
	 * アラームハンドラ
	 */
	syslog_0(LOG_NOTICE, "=== test start from ALARM ===");
	ercd = sta_alm(ALM, 0);
	check_ercd(ercd, E_OK);
	ttsp_wait_check_point(5);
	ttsp_check_point(6);

	/* [e][k]ディスパッチ禁止状態 */
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ercd = sta_alm(ALM, 0);
	check_ercd(ercd, E_OK);
	ttsp_wait_check_point(7);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	ttsp_check_point(8);


	/* 
	 * 周期ハンドラ
	 */
	syslog_0(LOG_NOTICE, "=== test start from CYCLIC ===");
	ercd = sta_cyc(CYC);
	check_ercd(ercd, E_OK);
	ttsp_wait_check_point(9);
	ercd = stp_cyc(CYC);
	check_ercd(ercd, E_OK);
	ttsp_check_point(10);

	/* [e][k]ディスパッチ禁止状態 */
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ercd = sta_cyc(CYC);
	check_ercd(ercd, E_OK);
	ttsp_wait_check_point(11);
	ercd = stp_cyc(CYC);
	check_ercd(ercd, E_OK);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	ttsp_check_point(12);


	/* 
	 * CPU例外ハンドラ
	 */
	syslog_0(LOG_NOTICE, "=== test start from EXCEPTION ===");
	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(13);
	ttsp_check_point(14);

	/* [d][j]割込み優先度マスク全解除でない状態 */
	ercd = chg_ipm(-1);
	check_ercd(ercd, E_OK);
	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(15);
	ercd = chg_ipm(TIPM_ENAALL);
	check_ercd(ercd, E_OK);
	ttsp_check_point(16);

	/* [e][k]ディスパッチ禁止状態 */
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(17);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	ttsp_check_point(18);


#ifdef TTSP_INTNO_B
	/* 
	 * 割込みハンドラ
	 */
	syslog_0(LOG_NOTICE, "=== test start from INTHDR ===");
	ttsp_int_raise(TTSP_INTNO_B);
	ttsp_wait_check_point(19);
	ttsp_check_point(20);

	/* [e][k]ディスパッチ禁止状態 */
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ttsp_int_raise(TTSP_INTNO_B);
	ttsp_wait_check_point(21);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	ttsp_check_point(22);
#else
	ttsp_check_point(19);
	ttsp_check_point(20);
	ttsp_check_point(21);
	ttsp_check_point(22);
#endif /* TTSP_INTNO_B */


#ifdef TTSP_INTNO_C
	/* 
	 * 割込みサービスルーチン
	 */
	syslog_0(LOG_NOTICE, "=== test start from ISR ===");
	ttsp_int_raise(TTSP_INTNO_C);
	ttsp_wait_check_point(23);
	ttsp_check_point(24);

	/* [e][k]ディスパッチ禁止状態 */
	ercd = dis_dsp();
	check_ercd(ercd, E_OK);
	ttsp_int_raise(TTSP_INTNO_C);
	ttsp_wait_check_point(25);
	ercd = ena_dsp();
	check_ercd(ercd, E_OK);
	ttsp_check_point(26);
#endif /* TTSP_INTNO_C */


	/* 全割込みロック状態でext_kerを発行できることの確認 */
	SIL_LOC_INT();
	ext_ker();
}

void texhdr(TEXPTN texptn, intptr_t exinf) {
	ttsp_check_point(3);
	all_test();
}

void almhdr(intptr_t exinf) {
	static int bootcnt = 0;

	bootcnt++;

	if (bootcnt == 1) {
		ttsp_check_point(5);
		all_test();
	}
	if (bootcnt == 2) {
		ttsp_check_point(7);
		part_test(DIS_DSP);
	}
}

void cychdr(intptr_t exinf) {
	static int bootcnt = 0;

	bootcnt++;

	if (bootcnt == 1) {
		ttsp_check_point(9);
		all_test();
	}
	if (bootcnt == 2) {
		ttsp_check_point(11);
		part_test(DIS_DSP);
	}
}

void inirtn(intptr_t exinf) {
	ttsp_initialize_test_lib();
	int_flag = false;

	syslog_0(LOG_NOTICE, "=== test start from INIRTN ===");

	/*「メモリ空間アクセス関数」の確認 */
	test_of_sil_mem();

	/* sns_ker()の確認 */
	test_of_sns_ker(sns_ker());

	/* [a]通常状態 */
	test_of_SIL_LOC_INT();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [a]: OK");
}

void terrtn(intptr_t exinf) {
	bool_t state;

	/* エラーが発生している場合は終了ルーチンを実行しない */
	state = ttsp_get_cp_state();
	if (state == false) {
		return;
	};

	syslog_0(LOG_NOTICE, "=== test start from TERRTN ===");

	/*「メモリ空間アクセス関数」の確認 */
	test_of_sil_mem();

	/* sns_ker()の確認 */
	test_of_sns_ker(sns_ker());

	/* [a]通常状態 */
	test_of_SIL_LOC_INT();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [a]: OK");

	/* すべて正常終了であればメッセージ表示 */
	state = ttsp_get_cp_state();
	if (state == true) {
		syslog_0(LOG_NOTICE, "All check points passed.");
	};
}

void exchdr(void* p_excinf) {
	static int bootcnt = 0;

	bootcnt++;

	ttsp_cpuexc_hook(TTSP_EXCNO_A, p_excinf);

	if (bootcnt == 1) {
		ttsp_check_point(13);
		all_test();
	}
	if (bootcnt == 2) {
		ttsp_check_point(15);
		part_test(CHG_IPM);
	}
	if (bootcnt == 3) {
		ttsp_check_point(17);
		part_test(DIS_DSP);
	}
}

#ifdef TTSP_INTNO_B
void inthdr(void) {
	static int bootcnt = 0;

	i_begin_int(TTSP_INTNO_B);
	ttsp_clear_int_req(TTSP_INTNO_B);

	bootcnt++;

	if (bootcnt == 1) {
		ttsp_check_point(19);
		all_test();
	}
	if (bootcnt == 2) {
		ttsp_check_point(21);
		part_test(DIS_DSP);
	}

	i_end_int(TTSP_INTNO_A);
}
#endif /* TTSP_INTNO_B */

#ifdef TTSP_INTNO_C
void isr(intptr_t exinf) {
	static int bootcnt = 0;

	ttsp_clear_int_req(TTSP_INTNO_C);

	bootcnt++;

	if (bootcnt == 1) {
		ttsp_check_point(23);
		all_test();
	}
	if (bootcnt == 2) {
		ttsp_check_point(25);
		part_test(DIS_DSP);
	}
}
#endif /* TTSP_INTNO_C */

void all_test(void) {
	ER ercd;

	/*「微少時間待ち」の確認 */
	if (!(sns_ctx())) {
		test_of_sil_dly_nse();
	}

	/*「メモリ空間アクセス関数」の確認 */
	test_of_sil_mem();

	/* 「sns_ker()発行」の確認 */
	test_of_sns_ker(sns_ker());

	/*
	 * 「全割込みロック状態の制御」の確認
	 */

	/* 事前条件にネストがないテスト */

	/* [a]通常状態 */
	test_of_SIL_LOC_INT();
	wait_raise_int();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [a]: OK");

	/* [b]全割込みロック状態 */
	{
		SIL_PRE_LOC;
		SIL_LOC_INT();
		test_of_SIL_LOC_INT();
		SIL_UNL_INT();
	}
	wait_raise_int();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [b]: OK");

	/* [c]CPUロック状態 */
	if (sns_ctx()) {
		ercd = iloc_cpu();
	} else {
		ercd = loc_cpu();
	}
	check_ercd(ercd, E_OK);
	test_of_SIL_LOC_INT();
	if (sns_ctx()) {
		ercd = iunl_cpu();
	} else {
		ercd = unl_cpu();
	}
	check_ercd(ercd, E_OK);
	wait_raise_int();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [c]: OK");

	/* [d]割込み優先度マスク全解除でない状態 */
	if (!(sns_ctx())) {
		ercd = chg_ipm(-1);
		check_ercd(ercd, E_OK);
		test_of_SIL_LOC_INT();
		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);
		wait_raise_int();
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [d]: OK");
	}

	/* [e]ディスパッチ禁止状態 */
	if (!(sns_ctx())) {
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);
		test_of_SIL_LOC_INT();
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);
		wait_raise_int();
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [e]: OK");
	}

	/* 事前条件にネストが存在するテスト */

	/* [h]SIL_LOC_INT() → SIL_LOC_INT() */
	{
		SIL_PRE_LOC;
		SIL_LOC_INT();
		{
			SIL_PRE_LOC;
			SIL_LOC_INT();
			test_of_SIL_LOC_INT();
			SIL_UNL_INT();
		}
		SIL_UNL_INT();
	}
	wait_raise_int();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [h]: OK");

	/* [i]loc_cpu()/iloc_cpu() → SIL_LOC_INT() */
	if (sns_ctx()) {
		ercd = iloc_cpu();
	} else {
		ercd = loc_cpu();
	}
	check_ercd(ercd, E_OK);
	{
		SIL_PRE_LOC;
		SIL_LOC_INT();
		test_of_SIL_LOC_INT();
		SIL_UNL_INT();
	}
	if (sns_ctx()) {
		ercd = iunl_cpu();
	} else {
		ercd = unl_cpu();
	}
	check_ercd(ercd, E_OK);
	wait_raise_int();
	syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [i]: OK");

	/* [j]chg_ipm(-1) → SIL_LOC_INT() */
	if (!(sns_ctx())) {
		ercd = chg_ipm(-1);
		check_ercd(ercd, E_OK);
		{
			SIL_PRE_LOC;
			SIL_LOC_INT();
			test_of_SIL_LOC_INT();
			SIL_UNL_INT();
		}
		ercd = chg_ipm(TIPM_ENAALL);
		check_ercd(ercd, E_OK);
		wait_raise_int();
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [j]: OK");
	}

	/* [k]dis_dsp() → SIL_LOC_INT() */
	if (!(sns_ctx())) {
		ercd = dis_dsp();
		check_ercd(ercd, E_OK);
		{
			SIL_PRE_LOC;
			SIL_LOC_INT();
			test_of_SIL_LOC_INT();
			SIL_UNL_INT();
		}
		ercd = ena_dsp();
		check_ercd(ercd, E_OK);
		wait_raise_int();
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [k]: OK");
	}
}

void part_test(E_TEST_TYPE test_type) {

	/* [d]割込み優先度マスク全解除でない状態 */
	/* [e]ディスパッチ禁止状態 */
	test_of_SIL_LOC_INT();
	wait_raise_int();
	if (test_type == CHG_IPM) {
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [d]: OK");
	}
	else if (test_type == DIS_DSP) {
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [e]: OK");
	}

	/* [j]chg_ipm(-1) → SIL_LOC_INT() */
	/* [k]dis_dsp() → SIL_LOC_INT() */
	{
		SIL_PRE_LOC;
		SIL_LOC_INT();
		test_of_SIL_LOC_INT();
		SIL_UNL_INT();
	}
	wait_raise_int();
	if (test_type == CHG_IPM) {
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [j]: OK");
	}
	else if (test_type == DIS_DSP) {
		syslog_0(LOG_NOTICE, "test_of_SIL_LOC_INT() [k]: OK");
	}
}

void inthdr_for_int_test(void) {
	i_begin_int(TTSP_INTNO_A);
	ttsp_clear_int_req(TTSP_INTNO_A);

	/* 割込みが入ったことを通知 */
	int_flag = true;

	i_end_int(TTSP_INTNO_A);
}

void test_of_sil_dly_nse(void) {
	ER ercd;
	SYSTIM system1, system2;

	ercd = get_tim(&system1);
	check_ercd(ercd, E_OK);
	sil_dly_nse(SIL_DLY_TIME);
	ercd = get_tim(&system2);
	check_ercd(ercd, E_OK);
	check_assert((system2 - system1) >= SIL_DLY_TIME_SUB);

	syslog_0(LOG_NOTICE, "test_of_sil_dly_nse()    : OK");
}

void test_of_sil_mem(void) {
	check_of_sil_mem();

	{
		SIL_PRE_LOC;
		SIL_LOC_INT();
		check_of_sil_mem();
		SIL_UNL_INT();
	}

	syslog_0(LOG_NOTICE, "test_of_sil_mem()        : OK");
}

void test_of_sns_ker(bool_t flag) {
	bool_t state;

	state = sns_ker();
	check_assert(state == flag);

	{
		SIL_PRE_LOC;
		SIL_LOC_INT();
		state = sns_ker();
		check_assert(state == flag);
		SIL_UNL_INT();
	}

	syslog_0(LOG_NOTICE, "test_of_sns_ker()        : OK");
}

void test_of_SIL_LOC_INT(void) {
	SIL_PRE_LOC;

	/* フラグ初期化 */
	int_flag = false;

	/* テスト対象のSIL関数発行 */
	SIL_LOC_INT();

	/* 割込みを発生 */
	ttsp_int_raise(TTSP_INTNO_A);

	/* 割込みが発生しないことを待つ */
	sil_dly_nse(TTSP_WAIT_NOT_RAISE_INT);

	/* フラグが更新されていないことを確認 */
	check_assert(int_flag == false);

	/* 元の状態に戻す */
	/* (b-2/h-2/i-2/j-2/k-2のテストを兼ねる) */
	SIL_UNL_INT();
}

void wait_raise_int(void) {
	ulong_t timeout = 0;

	/* 割込みが発生するのを待つ */
	while (int_flag == false) {
		timeout++;
		if (timeout > TTSP_LOOP_COUNT) {
			syslog_0(LOG_ERROR, "## wait_raise_int() caused a timeout.");
			ttsp_set_cp_state(false);
			ext_ker();
		}
		sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
	}
}

void check_of_sil_mem(void) {
	uint8_t   r8_data  = R_DATA8;
	uint16_t  r16_data = R_DATA16;
	uint32_t  r32_data = R_DATA32;
	uint8_t   w8_data  = W_DATA8;
	uint16_t  w16_data = W_DATA16;
	uint32_t  w32_data = W_DATA32;
	uint8_t   w8_temp  = CLEAR32;
	uint16_t  w16_temp = CLEAR32;
	uint32_t  w32_temp = CLEAR32;

	/* 8bit メモリ空間アクセス関数 */
	w8_temp = sil_reb_mem((void*)&r8_data);
	check_assert(w8_temp == R_DATA8);
	w8_temp = CLEAR32;

	sil_wrb_mem((void*)&w8_temp, w8_data);
	check_assert(w8_temp == W_DATA8);
	w8_temp = CLEAR32;

	/* 16bit メモリ空間アクセス関数 */
	w16_temp = sil_reh_mem((void*)&r16_data);
	check_assert(w16_temp == R_DATA16);
	w16_temp = CLEAR32;

	sil_wrh_mem((void*)&w16_temp, w16_data);
	check_assert(w16_temp == W_DATA16);
	w16_temp = CLEAR32;

#ifdef SIL_ENDIAN_LITTLE
	w16_temp = sil_reh_lem((void*)&r16_data);
#endif /* SIL_ENDIAN_LITTLE */
#ifdef SIL_ENDIAN_BIG
	w16_temp = sil_reh_bem((void*)&r16_data);
#endif /* SIL_ENDIAN_BIG */
	check_assert(w16_temp == R_DATA16);
	w16_temp = CLEAR32;

#ifdef SIL_ENDIAN_LITTLE
	sil_wrh_lem((void*)&w16_temp, w16_data);
#endif /* SIL_ENDIAN_LITTLE */
#ifdef SIL_ENDIAN_BIG
	sil_wrh_bem((void*)&w16_temp, w16_data);
#endif /* SIL_ENDIAN_BIG */
	check_assert(w16_temp == W_DATA16);
	w16_temp = CLEAR32;

	/* 32bit メモリ空間アクセス関数 */
	w32_temp = sil_rew_mem((void*)&r32_data);
	check_assert(w32_temp == R_DATA32);
	w32_temp = CLEAR32;

	sil_wrw_mem((void*)&w32_temp, w32_data);
	check_assert(w32_temp == W_DATA32);
	w32_temp = CLEAR32;

#ifdef SIL_ENDIAN_LITTLE
	w32_temp = sil_rew_lem((void*)&r32_data);
#endif /* SIL_ENDIAN_LITTLE */
#ifdef SIL_ENDIAN_BIG
	w32_temp = sil_rew_bem((void*)&r32_data);
#endif /* SIL_ENDIAN_BIG */
	check_assert(w32_temp == R_DATA32);
	w32_temp = CLEAR32;

#ifdef SIL_ENDIAN_LITTLE
	sil_wrw_lem((void*)&w32_temp, w32_data);
#endif /* SIL_ENDIAN_LITTLE */
#ifdef SIL_ENDIAN_BIG
	sil_wrw_bem((void*)&w32_temp, w32_data);
#endif /* SIL_ENDIAN_BIG */
	check_assert(w32_temp == W_DATA32);
	w32_temp = CLEAR32;
}
