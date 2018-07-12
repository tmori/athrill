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
	EXCNO excno;
	ID almid;
	ID cycid;
}PRC_INFO;

PRC_INFO prc_info[] = {
#ifdef TOPPERS_SYSTIM_LOCAL
	{TTSP_EXCNO_A,     ALM1, CYC1},
#if TNUM_PRCID >= 2
	{TTSP_EXCNO_PE2_A, ALM2, CYC2},
#endif /* TNUM_PRCID >= 2 */
#if TNUM_PRCID >= 3
	{TTSP_EXCNO_PE3_A, ALM3, CYC3},
#endif /* TNUM_PRCID >= 3 */
#if TNUM_PRCID >= 4
	{TTSP_EXCNO_PE4_A, ALM4, CYC4}
#endif /* TNUM_PRCID >= 4 */
#else /* TOPPERS_SYSTIM_GLOBAL */
	{TTSP_EXCNO_A,     ALM1, CYC1},
#if TNUM_PRCID >= 2
	{TTSP_EXCNO_PE2_A, ALM1, CYC1},
#endif /* TNUM_PRCID >= 2 */
#if TNUM_PRCID >= 3
	{TTSP_EXCNO_PE3_A, ALM1, CYC1},
#endif /* TNUM_PRCID >= 3 */
#if TNUM_PRCID >= 4
	{TTSP_EXCNO_PE4_A, ALM1, CYC1}
#endif /* TNUM_PRCID >= 4 */
#endif /* TOPPERS_SYSTIM_LOCAL */
};

void main_task(intptr_t exinf){
	uint_t i;
	ER ercd;
	EXCNO excno = prc_info[(int)exinf].excno;
	ID almid = prc_info[(int)exinf].almid;
	ID cycid = prc_info[(int)exinf].cycid;
	ID prcid;
	get_pid(&prcid);
	i = 0;

	ttsp_mp_check_point(prcid, 1);

	/* CPU例外発生元がタスク */

	/* CPU例外を起こす */
	ttsp_cpuexc_raise(excno);
	ttsp_mp_wait_check_point(prcid, 2);

	ttsp_mp_check_point(prcid, 3);

	/* CPU例外発生元がタスク例外処理ルーチン */

  	/* タスク例外許可状態にする */
	ercd = ena_tex();
	check_ercd(ercd, E_OK);
  
	/* タスク例外を起こす */
	ercd = ras_tex(TSK_SELF, 0x00000001);
	check_ercd(ercd, E_OK);
  
	ttsp_mp_check_point(prcid, 7);


#ifdef TOPPERS_SYSTIM_GLOBAL
	if (TOPPERS_SYSTIM_PRCID == prcid) {
#endif /* TOPPERS_SYSTIM_GLOBAL */

	/* CPU例外発生元がアラームハンドラ */

	/* アラームハンドラを起こす */
	ercd = sta_alm(almid, 0);
	check_ercd(ercd, E_OK);

	ttsp_mp_wait_check_point(prcid, 8);

	ttsp_mp_check_point(prcid, 11);


	/* CPU例外発生元が周期ハンドラ */

	/* 周期ハンドラを起こす */
	ercd = sta_cyc(cycid);
	check_ercd(ercd, E_OK);

	ttsp_mp_wait_check_point(prcid, 12);

	/* 周期ハンドラを終了させる */
	ercd = stp_cyc(cycid);
	check_ercd(ercd, E_OK);

#ifdef TOPPERS_SYSTIM_GLOBAL
	}
	if ((TOPPERS_MASTER_PRCID != TOPPERS_SYSTIM_PRCID) && (TOPPERS_MASTER_PRCID == prcid)) {
		for (i = 8; i <= 14; i++) {
			ttsp_mp_check_point(prcid, i);
		}
	}
#endif /* TOPPERS_SYSTIM_GLOBAL */

	ttsp_barrier_sync(1, TNUM_PRCID);

	if (TOPPERS_MASTER_PRCID == prcid) {
		ttsp_mp_check_finish(prcid, 15);
	}
}

void exc(void* p_excinf){
	static uint_t bootcnt[4] = {0};
	EXCNO excno;
	ID prcid;
	iget_pid(&prcid);

	bootcnt[prcid - 1]++;
	excno = prc_info[prcid - 1].excno;

	ttsp_cpuexc_hook(excno, p_excinf);

	if (bootcnt[prcid - 1] == 1) {
		syslog_2(LOG_NOTICE, "[TSK%d]ttsp_cpuexc_raise(0x%x) : OK", prcid, excno);
		ttsp_mp_check_point(prcid, 2);
	}
	if (bootcnt[prcid - 1] == 2) {
		syslog_2(LOG_NOTICE, "[TEX%d]ttsp_cpuexc_raise(0x%x) : OK", prcid, excno);
		ttsp_mp_check_point(prcid, 5);
	}
	if (bootcnt[prcid - 1] == 3) {
		syslog_2(LOG_NOTICE, "[ALM%d]ttsp_cpuexc_raise(0x%x) : OK", prcid, excno);
		ttsp_mp_check_point(prcid, 9);
	}
	if (bootcnt[prcid - 1] == 4) {
		syslog_2(LOG_NOTICE, "[CYC%d]ttsp_cpuexc_raise(0x%x) : OK", prcid, excno);
		ttsp_mp_check_point(prcid, 13);
	}
}

void tex(TEXPTN texptn, intptr_t exinf){
	EXCNO excno = prc_info[(int)exinf].excno;
	ID prcid;
	get_pid(&prcid);

	ttsp_mp_check_point(prcid, 4);

	ttsp_cpuexc_raise(excno);
	ttsp_mp_wait_check_point(prcid, 5);

	ttsp_mp_check_point(prcid, 6);
}

void alm(intptr_t exinf){
	EXCNO excno = prc_info[(int)exinf].excno;
	ID prcid;
	iget_pid(&prcid);

	ttsp_mp_check_point(prcid, 8);

	ttsp_cpuexc_raise(excno);
	ttsp_mp_wait_check_point(prcid, 9);

	ttsp_mp_check_point(prcid, 10);
}

void cyc(intptr_t exinf){
	EXCNO excno = prc_info[(int)exinf].excno;
	ID prcid;
	iget_pid(&prcid);

	ttsp_mp_check_point(prcid, 12);

	ttsp_cpuexc_raise(excno);
	ttsp_mp_wait_check_point(prcid, 13);

	ttsp_mp_check_point(prcid, 14);
}

void ttsp_test_lib_init(intptr_t exinf){
	ttsp_initialize_test_lib();
}
