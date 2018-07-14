/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2011 by Center for Embedded Computing Systems
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
 *  $Id: ttsp_target_test.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

#include "kernel_impl.h"
#include <sil.h>
#include "ttsp_target_test.h"

/*
 *  ティック更新の停止
 */
void
ttsp_target_stop_tick(void)
{
	uint32_t tmp;
	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	tmp &= ~SYSTIC_ENABLE;
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
}

/*
 *  ティック更新の再開
 */
void
ttsp_target_start_tick(void)
{
	uint32_t tmp;
	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	tmp |= SYSTIC_ENABLE;
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
}

/*
 *  ティックの更新
 */
void
ttsp_target_gain_tick(void)
{
	uint32_t tmp;
	SIL_PRE_LOC;
	
	SIL_LOC_INT();
	/* タイマ再開 */
	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	tmp |= SYSTIC_ENABLE;
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
	
	/* 割込みが発生するまで待つ */
	do{
		tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	}while((tmp & SYSTIC_COUNTFLAG) != SYSTIC_COUNTFLAG);

	/* タイマ停止 */
	tmp = sil_rew_mem((void *)SYSTIC_CONTROL_STATUS);
	tmp &= ~SYSTIC_ENABLE;
	sil_wrw_mem((void *)SYSTIC_CONTROL_STATUS, tmp);
	SIL_UNL_INT();
}

/*
 *  割込みの発生   
 */
void
ttsp_int_raise(INTNO intno){
	int tmp = intno - 16;
	sil_wrw_mem((void *)((uint32_t *)0xE000E200 + (tmp >> 5)),
				(1 << (tmp & 0x1f)));
}

/*
 *  CPU例外を発生させる命令
 */
__asm static void _raise_exception(void) {
	mcr p15, 0, r1, c2, c0, 0  
	bx   lr
}

/*
 *  CPU例外の発生
 */
void
ttsp_cpuexc_raise(EXCNO excno)
{
	if (excno == TTSP_EXCNO_A) {
		_raise_exception();
	}
}

/*
 *  割込み要求のクリア(テスト用：不要)
 */
void
ttsp_clear_int_req(INTNO intno)
{

}

/*
 *  CPU例外ハンドラの入り口処理
 */
void
ttsp_cpuexc_hook(EXCNO excno, void* p_excinf) {
	/* 戻りアドレスを設定 */
	*((uint32_t *)p_excinf + 8) = *((uint32_t *)p_excinf + 8) + 4;
}
