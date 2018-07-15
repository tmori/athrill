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
 *  $Id: ttsp_target_timer.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

/*
 *  タイマドライバ（Nios2用）
 */

#include "kernel/kernel_impl.h"
#include "time_event.h"
#include <sil.h>
#include "target_timer.h"
#include "ttsp_target_test.h"


/*
 *  ttsp_target_timer.cリンク確認用グローバル変数
 */
uint_t include_ttsp_target_timer_to_KERNEL_COBJS = 0;

/*
 *  タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK cyc = TO_CLOCK(TIC_NUME, TIC_DENO);
	uint32_t base = target_sys_clk_timer_base_table[x_prc_index()];

	/* タイマーストップ */
	sil_wrw_iop((void*)(base + AVALON_TIM_CONTROL), AVALON_TIM_CONTROL_STOP);
	/* タイムアウトステータスクリア */
	sil_wrw_iop((void*)(base + AVALON_TIM_STATUS), 0x00);

	assert(cyc <= MAX_CLOCK);                          /* タイマ上限値のチェック */
	sil_wrw_iop((void*)(base + AVALON_TIM_PERIODL), (cyc & 0xffff)); /* カウンターセット 下位16bit */
	sil_wrw_iop((void*)(base + AVALON_TIM_PERIODH), (cyc >> 16));    /* カウンターセット 上位16bit */

	/*
	 * タイマースタート，オートリロード，割込み許可
	 */
	sil_wrw_iop((void*)(base + AVALON_TIM_CONTROL), AVALON_TIM_CONTROL_START
				|AVALON_TIM_CONTROL_COUNT|AVALON_TIM_CONTROL_ITO);
}

/*
 *  タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	uint32_t base = target_sys_clk_timer_base_table[x_prc_index()];

	/* タイマ停止 */
	sil_wrw_iop((void*)(base + AVALON_TIM_CONTROL), AVALON_TIM_CONTROL_STOP);

	/* 割込み要求のクリア */
	sil_wrw_iop((void*)(base + AVALON_TIM_STATUS), 0x00);
}

/*
 *  タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	uint32_t base = target_sys_clk_timer_base_table[x_prc_index()];
	uint32_t intno = target_sys_clk_timer_intno_table[x_prc_index()];

	/* TOビットのクリア */
	sil_wrw_iop((void*)(base + AVALON_TIM_STATUS), 0x00);

	i_begin_int(intno);
	/* タイムティックの供給 */
	if (ttsp_timer_handler_begin_hook()) {
		signal_time();                    /* タイムティックの供給 */
		ttsp_timer_handler_end_hook();
	}	
	i_end_int(intno);
}
