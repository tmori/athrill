/*
 *  TOPPERS/HRP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      High Reliable system Profile Kernel
 *	
 *	Copyright (C) 2011 by Embedded and Real-Time Systems Laboratory
 *				Graduate School of Information Science, Nagoya Univ., JAPAN
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
 */

/*
 *	タイマドライバ（SH2AW用）
 *	
 *	CMT0を使用
 */

#include "kernel_impl.h"
#include <sil.h>
#include "target_timer.h"
#include "time_event.h"
#include "overrun.h"

/*
 *	タイマの起動処理
 */
void
target_timer_initialize(intptr_t exinf)
{
	CLOCK	 cyc = TO_CLOCK(TIC_NUME, TIC_DENO) - 1;

	/*
	 *	タイマ周期を設定し，タイマの動作を開始する．
	 */
	/* CMT0停止 */
	sil_wrh_mem((void *)CM01STR,
				(sil_reh_mem((void *)CM01STR) & ~CMT0_STR));

	/* タイマ上限値のチェック */
	assert(cyc <= MAX_CLOCK);

	/* 分周比設定、割込み許可 */
	sil_wrh_mem((void *)CMT0CR, (CMSR_CKS_DIV32	| CMCR_CMIE));

	/* コンペアマッチカウントをセット */
	sil_wrh_mem((void *)CMT0CMSE, (uint16_t)cyc);
	
    /* カウンタをリセット */
	sil_wrh_mem((void *)CMT0CNT, (uint16_t)0U);

	/* 割込み要求をクリア */
    x_clear_int(INTNO_TIMER);
	
    /* CMT0スタート */
	sil_wrh_mem((void*)CM01STR,
				(sil_reh_mem((void*)CM01STR) | CMT0_STR));

}

/*
 *	タイマの停止処理
 */
void
target_timer_terminate(intptr_t exinf)
{
	/* タイマを停止 */
	sil_wrh_mem((void *)CM01STR,
				(sil_reh_mem((void *)CM01STR) & ~CMT0_STR));
	
	/* 割込み要求をクリア */
    x_clear_int(INTNO_TIMER);

}

/*
 *	タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	/* 割込み要求をクリア */
    x_clear_int(INTNO_TIMER);

	i_begin_int(INTNO_TIMER);
	signal_time();					  /* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}

#ifdef TOPPERS_TARGET_SUPPORT_OVRHDR
/*
 *	オーバランタイマの初期化処理
 *
 *	タイマを初期化する．
 */
void
target_ovrtimer_initialize(intptr_t exinf)
{
	/*
	 *	タイマ周期を設定する．
	 */
	/* CMT1停止 */
	sil_wrh_mem((void *)CM01STR,
				(sil_reh_mem((void *)CM01STR) & ~CMT1_STR));

	/* 分周比設定、割込み許可 */
	sil_wrh_mem((void *)CMT1CR, (CMSR_CKS_DIV512 | CMCR_CMIE));

}

/*
 *	オーバランタイマの終了処理
 *
 *	タイマの動作を終了する．
 */
void
target_ovrtimer_terminate(intptr_t exinf)
{
	/* タイマを停止 */
	sil_wrh_mem((void *)CM01STR,
				(sil_reh_mem((void *)CM01STR) & ~CMT1_STR));
	
	/* 割込み要求をクリア */
    x_clear_int(INTNO_TIMER);

}

/*
 *	オーバランタイマの起動処理
 *
 *	タイマを起動する．
 */
void
target_ovrtimer_start(OVRTIM ovrtim)
{
	CLOCK	 cyc = OVRTIMER_CLOCK(ovrtim) - 1;

	/*
	 *	タイマ周期を設定し，タイマの動作を開始する．
	 */
	/* タイマ上限値のチェック */
	assert(cyc <= MAX_CLOCK);

	/* コンペアマッチカウントをセット */
	sil_wrh_mem((void *)CMT1CMSE, (uint16_t)cyc);
	
    /* カウンタをリセット */
	sil_wrh_mem((void *)CMT1CNT, (uint16_t)0U);

	/* 割込み要求をクリア */
    x_clear_int(INTNO_TIMER);
	
    /* CMT1スタート */
	sil_wrh_mem((void*)CM01STR,
				(sil_reh_mem((void*)CM01STR) | CMT1_STR));

}

/*
 *	オーバランタイマの停止処理
 *
 *	タイマの動作を停止させる．
 */
OVRTIM
target_ovrtimer_stop(void)
{
    OVRTIM rest;

	/* タイマを停止 */
	sil_wrh_mem((void *)CM01STR,
				(sil_reh_mem((void *)CM01STR) & ~CMT1_STR));
	
	/* 割込み要求をクリア */
    x_clear_int(INTNO_TIMER);

	rest = ((OVRTIM)sil_reh_mem((void*)CMT1CMSE) - (OVRTIM)sil_reh_mem((void*)CMT1CNT));

    if(rest < 0){
        return 1;
    } else {
        return (OVRTIMER_USEC(rest + 1));
    }
}

/*
 *	オーバランタイマの残り時間の読出し
 */
OVRTIM
target_ovrtimer_get_current(void)
{
    OVRTIM rest;

	rest = ((OVRTIM)sil_reh_mem((void*)CMT1CMSE) - (OVRTIM)sil_reh_mem((void*)CMT1CNT));

    if(rest < 0){
        return 0;
    } else {
        return (OVRTIMER_USEC(rest + 1));
    }
}

/*
 *	オーバランタイマ割込みハンドラ
 */
void
target_ovrtimer_handler(void)
{
	i_begin_int(INTNO_OVRTIMER);
	call_ovrhdr();
	i_end_int(INTNO_OVRTIMER);
}

#endif /* TOPPERS_TARGET_SUPPORT_OVRHDR */


