/*
 *	TOPPERS/HRP Kernel
 *		Toyohashi Open Platform for Embedded Real-Time Systems/
 *		High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2007 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
 *	Copyright (C) 2010-2011 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
 *	
 *	上記著作権者は，以下の(1)〜(4)の条件を満たす場合に限り，本ソフトウェ
 *	ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *	変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *	(1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *		権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *		スコード中に含まれていること．
 *	(2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *		用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *		者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *		の無保証規定を掲載すること．
 *	(3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *		用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *		と．
 *	  (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *		  作権表示，この利用条件および下記の無保証規定を掲載すること．
 *	  (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *		  報告すること．
 *	(4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *		害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *		また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *		由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *		免責すること．
 *	
 *	本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *	よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *	に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *	アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *	の責任を負わない．
 *	
 *	$Id: prc_cmt.c 2144 2011-06-29 07:57:57Z mit-kimai $
 */

/*
 *	タイマドライバ（SH12A用）
 *	
 *	CMT0を使用
 */

#include "kernel_impl.h"
#include "time_event.h"
#include <sil.h>
#include "target_timer.h"

/*
 *  タイマスタート
 */
Inline void
target_timer_start(void)
{
	/* CMT0スタート */
	sil_orh_reg((uint16_t *)CMSTR_h, CMSTR_STR0);
}

/*
 *  タイマ停止
 */
Inline void
target_timer_stop(void)
{
	/* CMT0停止 */
	sil_anh_reg((uint16_t *)CMSTR_h, ~CMSTR_STR0);
}

/*
 *  タイマ割込み要求をクリア
 */
Inline void
target_timer_clear_int(void)
{
	sil_anh_reg((uint16_t *)CMCSR_0_h, ~CMCSR_CMF);
}

/*
 *	タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK	 cyc = TO_CLOCK(TIC_NUME, TIC_DENO);

	/*
	 *	タイマ周期を設定し，タイマの動作を開始する．
	 */
	target_timer_stop();		/* タイマ停止 */

	assert(cyc <= MAX_CLOCK);	/* タイマ上限値のチェック */

	/* 分周比設定(PCLOCK/8)、割込み許可 */
	sil_wrh_mem((uint16_t *)CMCSR_0_h, (CMCSR_CKS | CMCSR_CMIE));

	/*constantレジスタをセット */
	sil_wrh_mem((uint16_t *)CMCOR_0_h, cyc);

	target_timer_clear_int();	/* 割込み要求をクリア */
	target_timer_start();		/* タイマスタート */
}

/*
 *	タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	target_timer_stop();		/* タイマ停止 */
	target_timer_clear_int();	/* 割込み要求をクリア */
}

/*
 *	タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	target_timer_clear_int();	/* 割込み要求をクリア */
	
	i_begin_int(INTNO_TIMER);
	signal_time();				/* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}
