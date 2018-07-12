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

void main_task(intptr_t exinf){
	ER ercd;

	ttsp_initialize_test_lib();

	ttsp_check_point(1);


	/* CPU例外発生元がタスク */

	/* CPU例外を起こす */
	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(2);

	ttsp_check_point(3);


	/* CPU例外発生元がタスク例外処理ルーチン */

  	/* タスク例外許可状態にする */
	ercd = ena_tex();
	check_ercd(ercd, E_OK);

	/* タスク例外を起こす */
	ercd = ras_tex(TSK_SELF, 0x00000001);
	check_ercd(ercd, E_OK);

	ttsp_check_point(7);


	/* CPU例外発生元がアラームハンドラ */

	/* アラームハンドラを起こす */
	ercd = sta_alm(ALM1, 0);
	check_ercd(ercd, E_OK);

	dly_tsk(10);

	ttsp_check_point(11);


	/* CPU例外発生元が周期ハンドラ */

	/* 周期ハンドラを起こす */
	ercd = sta_cyc(CYC1);
	check_ercd(ercd, E_OK);

	dly_tsk(10);

	/* 周期ハンドラを終了させる */
	ercd = stp_cyc(CYC1);
	check_ercd(ercd, E_OK);

	ttsp_check_finish(15);
}

void exception_ttsp_excno_a(void* p_excinf){
	static uint_t bootcnt = 0;

	bootcnt++;

	ttsp_cpuexc_hook(TTSP_EXCNO_A, p_excinf);

	if (bootcnt == 1){
		syslog_0(LOG_NOTICE, "[TSK]ttsp_cpuexc_raise(TTSP_EXCNO_A) : OK");

		ttsp_check_point(2);
	}
	if(bootcnt == 2){
		syslog_0(LOG_NOTICE, "[TEX]ttsp_cpuexc_raise(TTSP_EXCNO_A) : OK");

		ttsp_check_point(5);
	}
	if(bootcnt == 3){
		syslog_0(LOG_NOTICE, "[ALM]ttsp_cpuexc_raise(TTSP_EXCNO_A) : OK");

		ttsp_check_point(9);
	}
	if(bootcnt == 4){
		syslog_0(LOG_NOTICE, "[CYC]ttsp_cpuexc_raise(TTSP_EXCNO_A) : OK");

		ttsp_check_point(13);
	}
}

void texhdr_tex(TEXPTN texptn, intptr_t exinf){

	ttsp_check_point(4);

	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(5);

	ttsp_check_point(6);
}

void almhdr_alm(intptr_t exinf){

	ttsp_check_point(8);

	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(9);

	ttsp_check_point(10);
}

void cychdr_cyc(intptr_t exinf){

	ttsp_check_point(12);

	ttsp_cpuexc_raise(TTSP_EXCNO_A);
	ttsp_wait_check_point(13);

	ttsp_check_point(14);
}
