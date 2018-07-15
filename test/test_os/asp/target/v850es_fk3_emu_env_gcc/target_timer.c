/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel  
 * 
 *  Copyright (C) 2010 by Meika Sugimoto
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
 */

#include "kernel_impl.h"
#include "target_timer.h"
#include "time_event.h"

/*
 *	カーネルタイマの初期化
 *
 *	TMMを使用する．
 */
void target_timer_initialize(intptr_t exinf)
{
	/* タイマ停止，タイマ割込み要求のクリア */
	clr_bit(7 , TM0CTL0);
	x_clear_int(INTNO_TIMER);
	
	/* クロック分周比の設定，20MHzが供給されるので4分周=5MHzにする */
	sil_wrb_mem((void *)TM0CTL0 , 2u);
	/* コンペア値の設定50000カウントで1msec， */
	sil_wrh_mem((void *)TM0CMP0 , TIMER_CLOCK - 1);
	
	/* タイマ開始 */
	set_bit(7 , TM0CTL0);
}

void target_timer_terminate(intptr_t exinf)
{
	/* タイマ停止，タイマ割込み要求のクリアと割込み禁止 */
	clr_bit(7 , TM0CTL0);
	x_disable_int(INTNO_TIMER);
	x_clear_int(INTNO_TIMER);
}

/*
 *  タイマ割込みハンドラ
 */
void
target_timer_handler(void)
{
	i_begin_int(INTNO_TIMER);
	signal_time();					/* タイムティックの供給 */
	i_end_int(INTNO_TIMER);
}

