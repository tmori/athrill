/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2009-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2009-2011 by Digital Craft Inc.
 *  Copyright (C) 2009-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2009-2011 by FUJISOFT INCORPORATED
 *  Copyright (C) 2009-2010 by Mitsuhiro Matsuura
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
 *  $Id: out.c 24 2015-12-09 06:00:31Z ertl-toshinaga $
 */
#include <kernel.h>
#include <t_syslog.h>
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include "ttsp_test_lib.h"
#include "out.h"

extern const SIZE   _kernel_istksz_table[TNUM_PRCID];
extern STK_T *const _kernel_istk_table[TNUM_PRCID];

void main_task(intptr_t exinf){
	syslog_0(LOG_NOTICE,"FMP_staticAPI_DEF_ICS_F_c: Start");

	ttsp_mp_check_point(1, 1);

	ttsp_mp_wait_check_point(2, 2);

	syslog_0(LOG_NOTICE,"FMP_staticAPI_DEF_ICS_F_c: OK");

	ttsp_mp_check_finish(1, 2);
}
void task1(intptr_t exinf){
	ttsp_mp_wait_check_point(1, 1);

	ttsp_cpuexc_raise(TTSP_EXCNO_PE2_A);
}
void exc1(void* p_excinf){
	ER ercd;

	ttsp_cpuexc_hook(TTSP_EXCNO_PE2_A, p_excinf);

	ttsp_mp_check_point(2, 1);

	check_assert(&ercd > (ER*)((char *)_kernel_istk_table[1]));							/* ローカル変数がスタックの領域内にあるか確認 */
	check_assert(&ercd < (ER*)((char *)_kernel_istk_table[1] + _kernel_istksz_table[1]));	/* ローカル変数がスタックの領域内にあるか確認 */

	ttsp_mp_check_point(2, 2);
}
void ttsp_test_lib_init(intptr_t exinf){
	ttsp_initialize_test_lib();
}
