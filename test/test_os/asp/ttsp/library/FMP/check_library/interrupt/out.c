/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
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

typedef struct{
	INTNO intno_a;
	INTNO intno_b;
	INTNO intno_c;
}PRC_INFO;

typedef struct{
	INTNO intno_d;
	INTNO intno_e;
	INTNO intno_f;
}PRC_INFO2;

PRC_INFO prc_info[] = {
#ifdef TTSP_INTNO_B
#ifdef TTSP_INTNO_C
	{TTSP_INTNO_A, TTSP_INTNO_B, TTSP_INTNO_C},
#else
	{TTSP_INTNO_A, TTSP_INTNO_B, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_C */
#else
	{TTSP_INTNO_A, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_B */

#if TNUM_PRCID >= 2
#ifdef TTSP_INTNO_PE2_B
#ifdef TTSP_INTNO_PE2_C
	{TTSP_INTNO_PE2_A, TTSP_INTNO_PE2_B, TTSP_INTNO_PE2_C},
#else
	{TTSP_INTNO_PE2_A, TTSP_INTNO_PE2_B, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE2_C */
#else
	{TTSP_INTNO_PE2_A, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE2_B */
#endif /* TNUM_PRCID >= 2 */

#if TNUM_PRCID >= 3
#ifdef TTSP_INTNO_PE3_B
#ifdef TTSP_INTNO_PE3_C
	{TTSP_INTNO_PE3_A, TTSP_INTNO_PE3_B, TTSP_INTNO_PE3_C},
#else
	{TTSP_INTNO_PE3_A, TTSP_INTNO_PE3_B, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE3_C */
#else
	{TTSP_INTNO_PE3_A, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE3_B */
#endif /* TNUM_PRCID >= 3 */

#if TNUM_PRCID >= 4
#ifdef TTSP_INTNO_PE4_B
#ifdef TTSP_INTNO_PE4_C
	{TTSP_INTNO_PE4_A, TTSP_INTNO_PE4_B, TTSP_INTNO_PE4_C}
#else
	{TTSP_INTNO_PE4_A, TTSP_INTNO_PE4_B, TTSP_INVALID_INTNO}
#endif /* TTSP_INTNO_PE4_C */
#else
	{TTSP_INTNO_PE4_A, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO}
#endif /* TTSP_INTNO_PE4_B */
#endif /* TNUM_PRCID >= 4 */
};

PRC_INFO2 prc_info2[] = {
#ifdef TTSP_INTNO_D
#ifdef TTSP_INTNO_E
#ifdef TTSP_INTNO_F
	{TTSP_INTNO_D, TTSP_INTNO_E, TTSP_INTNO_F},
#else
	{TTSP_INTNO_D, TTSP_INTNO_E, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_F */
#else
	{TTSP_INTNO_D, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_E */
#else
	{TTSP_INVALID_INTNO, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_D */

#if TNUM_PRCID >= 2
#ifdef TTSP_INTNO_PE2_D
#ifdef TTSP_INTNO_PE2_E
#ifdef TTSP_INTNO_PE2_F
	{TTSP_INTNO_PE2_D, TTSP_INTNO_PE2_E, TTSP_INTNO_PE2_F},
#else
	{TTSP_INTNO_PE2_D, TTSP_INTNO_PE2_E, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE2_F */
#else
	{TTSP_INTNO_PE2_D, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE2_E */
#else
	{TTSP_INVALID_INTNO, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE2_D */
#endif /* TNUM_PRCID >= 2 */

#if TNUM_PRCID >= 3
#ifdef TTSP_INTNO_PE3_D
#ifdef TTSP_INTNO_PE3_E
#ifdef TTSP_INTNO_PE3_F
	{TTSP_INTNO_PE3_D, TTSP_INTNO_PE3_E, TTSP_INTNO_PE3_F},
#else
	{TTSP_INTNO_PE3_D, TTSP_INTNO_PE3_E, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE3_F */
#else
	{TTSP_INTNO_PE3_D, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE3_E */
#else
	{TTSP_INVALID_INTNO, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO},
#endif /* TTSP_INTNO_PE3_D */
#endif /* TNUM_PRCID >= 3 */

#if TNUM_PRCID >= 4
#ifdef TTSP_INTNO_PE4_D
#ifdef TTSP_INTNO_PE4_E
#ifdef TTSP_INTNO_PE4_F
	{TTSP_INTNO_PE4_D, TTSP_INTNO_PE4_E, TTSP_INTNO_PE4_F}
#else
	{TTSP_INTNO_PE4_D, TTSP_INTNO_PE4_E, TTSP_INVALID_INTNO}
#endif /* TTSP_INTNO_PE4_F */
#else
	{TTSP_INTNO_PE4_D, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO}
#endif /* TTSP_INTNO_PE4_E */
#else
	{TTSP_INVALID_INTNO, TTSP_INVALID_INTNO, TTSP_INVALID_INTNO}
#endif /* TTSP_INTNO_PE4_D */
#endif /* TNUM_PRCID >= 4 */
};

void main_task(intptr_t exinf){
	ER ercd;
	INTNO intno_a;
	INTNO intno_b;
	INTNO intno_d;
	INTNO intno_e;
	INTNO intno_f;
	uint_t finish_no;
	ID prcid;

	get_pid(&prcid);
	intno_a = prc_info[prcid - 1].intno_a;
	intno_b = prc_info[prcid - 1].intno_b;
	intno_d = prc_info2[prcid - 1].intno_d;
	intno_e = prc_info2[prcid - 1].intno_e;
	intno_f = prc_info2[prcid - 1].intno_f;

	ttsp_mp_check_point(prcid, 1);

	/* 実行中のタスクから割込みを発生 */

	/* 割込みを起こす */
	ttsp_int_raise(intno_a);
	ttsp_mp_wait_check_point(prcid, 2);

	ttsp_mp_check_point(prcid, 3);


	/* 実行中のタスク例外処理ルーチンから割込みを発生 */

	/* タスク例外許可状態にする */
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	/* タスク例外を起こす */
	ercd = ras_tex(TSK_SELF, 0x00000001);
	check_ercd(ercd, E_OK);

	ttsp_mp_check_point(prcid, 7);

	/* 割込みサービスルーチン */
	if (intno_d != TTSP_INVALID_INTNO) {
		ttsp_int_raise(intno_d);
		ttsp_mp_wait_check_point(prcid, 8);
		if (intno_e != TTSP_INVALID_INTNO) {
			ttsp_int_raise(intno_e);
			ttsp_mp_wait_check_point(prcid, 9);
			if (intno_f != TTSP_INVALID_INTNO) {
				ttsp_int_raise(intno_f);
				ttsp_mp_wait_check_point(prcid, 10);
			} else {
				ttsp_mp_check_point(prcid, 10);
			}
		} else {
			ttsp_mp_check_point(prcid, 9);
			ttsp_mp_check_point(prcid, 10);
		}
	} else {
		ttsp_mp_check_point(prcid, 8);
		ttsp_mp_check_point(prcid, 9);
		ttsp_mp_check_point(prcid, 10);
	}

	ttsp_mp_check_point(prcid, 11);

	/* 優先度が異なる割込みを、低→中→高の順で起こす */
	finish_no = 11;
	if (intno_b != TTSP_INVALID_INTNO) {
		/* 割込みを起こす */
		ttsp_int_raise(intno_a);
		ttsp_mp_wait_check_point(prcid, 12);

		finish_no = 17;
	}

	ttsp_barrier_sync(1, TNUM_PRCID);

	if (TOPPERS_MASTER_PRCID == (prcid)){
		ttsp_mp_check_finish(prcid, finish_no);
	}
}

void texhdr_tex(TEXPTN texptn, intptr_t exinf){
	INTNO intno;
	ID prcid;

	get_pid(&prcid);
	intno = prc_info[prcid - 1].intno_a;

	ttsp_mp_check_point(prcid, 4);

	/* 割込みを起こす */
	ttsp_int_raise(intno);
	ttsp_mp_wait_check_point(prcid, 5);

	ttsp_mp_check_point(prcid, 6);
}

void inthdr_ttsp_intno_a(void){
	INTNO intno_a;
	INTNO intno_b;
	ID prcid;
	static uint_t bootcnt[4] = {0};

	iget_pid(&prcid);

	bootcnt[prcid - 1]++;
	intno_a = prc_info[prcid - 1].intno_a;
	intno_b = prc_info[prcid - 1].intno_b;

	i_begin_int(intno_a);
	ttsp_clear_int_req(intno_a);

	if (bootcnt[prcid - 1] == 1) {
		ttsp_mp_check_point(prcid, 2);
		syslog_2(LOG_NOTICE, "[INTNO_A:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_a);
	}
	if (bootcnt[prcid - 1] == 2) {
		ttsp_mp_check_point(prcid, 5);
		syslog_2(LOG_NOTICE, "[INTNO_A:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_a);
	}
	if (bootcnt[prcid - 1] == 3) {
		ttsp_mp_check_point(prcid, 12);
		syslog_2(LOG_NOTICE, "[INTNO_A:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_a);

		/* 割込みを起こす */
		ttsp_int_raise(intno_b);
		ttsp_mp_wait_check_point(prcid, 13);

		ttsp_mp_check_point(prcid, 16);
	}

	i_end_int(intno_a);
}

#if defined(TTSP_INTNO_B) || defined(TTSP_INTNO_PE2_B) || defined(TTSP_INTNO_PE3_B) || defined(TTSP_INTNO_PE4_B)
void inthdr_ttsp_intno_b(void){
	INTNO intno_b;
	INTNO intno_c;
	ID prcid;

	iget_pid(&prcid);
	intno_b = prc_info[prcid - 1].intno_b;
	intno_c = prc_info[prcid - 1].intno_c;

	i_begin_int(intno_b);
	ttsp_clear_int_req(intno_b);

	ttsp_mp_check_point(prcid, 13);
	syslog_2(LOG_NOTICE, "[INTNO_B:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_b);

	if (intno_c != TTSP_INVALID_INTNO) {
		/* 割込みを起こす */
		ttsp_int_raise(intno_c);
		ttsp_mp_wait_check_point(prcid, 14);
	} else {
		ttsp_mp_check_point(prcid, 14);
	}

	ttsp_mp_check_point(prcid, 15);

	i_end_int(intno_b);
}
#endif /* defined(TTSP_INTNO_B) || defined(TTSP_INTNO_PE2_B) || defined(TTSP_INTNO_PE3_B) || defined(TTSP_INTNO_PE4_B) */

#if defined(TTSP_INTNO_C) || defined(TTSP_INTNO_PE2_C) || defined(TTSP_INTNO_PE3_C) || defined(TTSP_INTNO_PE4_C)
void inthdr_ttsp_intno_c(void){
	INTNO intno_c;
	ID prcid;

	iget_pid(&prcid);
	intno_c = prc_info[prcid - 1].intno_c;

	i_begin_int(intno_c);
	ttsp_clear_int_req(intno_c);

	ttsp_mp_check_point(prcid, 14);
	syslog_2(LOG_NOTICE, "[INTNO_C:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_c);

	i_end_int(intno_c);
}
#endif /* defined(TTSP_INTNO_C) || defined(TTSP_INTNO_PE2_C) || defined(TTSP_INTNO_PE3_C) || defined(TTSP_INTNO_PE4_C) */

#if defined(TTSP_INTNO_D) || defined(TTSP_INTNO_PE2_D) || defined(TTSP_INTNO_PE3_D) || defined(TTSP_INTNO_PE4_D)
void isr_ttsp_intno_d(intptr_t exinf){
	INTNO intno_d;
	ID prcid;

	iget_pid(&prcid);
	intno_d = prc_info2[prcid - 1].intno_d;

	ttsp_clear_int_req(intno_d);

	ttsp_mp_check_point(prcid, 8);
	syslog_2(LOG_NOTICE, "[INTNO_D:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_d);
}
#endif /* defined(TTSP_INTNO_D) || defined(TTSP_INTNO_PE2_D) || defined(TTSP_INTNO_PE3_D) || defined(TTSP_INTNO_PE4_D) */

#if defined(TTSP_INTNO_E) || defined(TTSP_INTNO_PE2_E) || defined(TTSP_INTNO_PE3_E) || defined(TTSP_INTNO_PE4_E)
void isr_ttsp_intno_e(intptr_t exinf){
	INTNO intno_e;
	ID prcid;

	iget_pid(&prcid);
	intno_e = prc_info2[prcid - 1].intno_e;

	ttsp_clear_int_req(intno_e);

	ttsp_mp_check_point(prcid, 9);
	syslog_2(LOG_NOTICE, "[INTNO_E:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_e);
}
#endif /* defined(TTSP_INTNO_E) || defined(TTSP_INTNO_PE2_E) || defined(TTSP_INTNO_PE3_E) || defined(TTSP_INTNO_PE4_E) */

#if defined(TTSP_INTNO_F) || defined(TTSP_INTNO_PE2_F) || defined(TTSP_INTNO_PE3_F) || defined(TTSP_INTNO_PE4_F)
void isr_ttsp_intno_f(intptr_t exinf){
	INTNO intno_f;
	ID prcid;

	iget_pid(&prcid);
	intno_f = prc_info2[prcid - 1].intno_f;

	ttsp_clear_int_req(intno_f);

	ttsp_mp_check_point(prcid, 10);
	syslog_2(LOG_NOTICE, "[INTNO_F:PE%d]ttsp_int_raise(0x%x) : OK", prcid, intno_f);
}
#endif /* defined(TTSP_INTNO_F) || defined(TTSP_INTNO_PE2_F) || defined(TTSP_INTNO_PE3_F) || defined(TTSP_INTNO_PE4_F) */

void ttsp_test_lib_init(intptr_t exinf){
	ttsp_initialize_test_lib();
}
