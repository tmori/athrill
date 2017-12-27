/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2014 by Center for Embedded Computing Systems
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
 *  本ソフトウェアは，AUTOSAR（AUTomotive Open System ARchitecture）仕
 *  様に基づいている．上記の許諾は，AUTOSARの知的財産権を許諾するもので
 *  はない．AUTOSARは，AUTOSAR仕様に基づいたソフトウェアを商用目的で利
 *  用する者に対して，AUTOSARパートナーになることを求めている．
 *
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 *
 *  @(#) $Id: ostm.c 164 2015-06-03 01:22:29Z t_ishikawa $
 */

#include "kernel_impl.h"
#include "target_timer.h"
#include "ostm.h"
#include "Os_Lcfg.h"

/*
 *  現在のシステム時刻（単位: ミリ秒）
 *
 *  厳密には，前のタイムティックのシステム時刻
 */
SystemTimeMsType current_time;

/*
 *	内部関数プロトタイプ宣言
 */
ISR(target_timer_hdr);

/*
 *  マイクロ秒単位での時刻を取得
 */
SystemTimeUsType
get_tim_utime(void)
{
	SystemTimeUsType	utime;
	SystemTimeMsType	mtime;
	TickType			clock1, clock2;
	boolean				ireq;
	SIL_PRE_LOC;

	SIL_LOC_INT();
	mtime = current_time;
	clock1 = target_timer_get_current();
	ireq = target_timer_probe_int();
	clock2 = target_timer_get_current();
	SIL_UNL_INT();
	utime = ((SystemTimeUsType) mtime) * 1000U;

	if ((ireq != FALSE) && (clock2 >= clock1)) {
		/*
		 *  割込みが入っており，clock2の方がclock1の方が大きいまたは等しい場合は，
		 *  current_time が割込みにより更新されていないかつ，clock1はオーバフロー後
		 *  の値であるため，utimeに1m追加する．clock1の読み込み，割込みのチェック，
		 *  clock2の読み込みが1μs以下で実行可能なプロセッサも存在するため，等号付き
		 *  で比較している．
		 */
		utime += 1000U;
	}
	utime += TO_USEC(clock1);

	return(utime);
}

/*
 *  OSTMの初期化
 */
void
target_timer_initialize(void)
{
	/* 停止 */
	sil_wrb_mem((void *) (OSTM_TT_B), 0x01);

	/* インターバル・タイマ・モード */
	sil_wrb_mem((void *) (OSTM_CTL_B), 0x00);

	/* コンペアレジスタの設定 */
	sil_wrw_mem((void *) (OSTM_CMP_W), TIMER_CLOCK);

	/* スタート */
	sil_wrb_mem((void *) (OSTM_TS_B), 0x01);
}

/*
 *  OSTMの停止
 */
void
target_timer_terminate(void)
{
	/* 停止 */
	sil_wrb_mem((void *) (OSTM_TT_B), 0x01);

	/* 割込みの禁止とクリア */
	x_disable_int(OSTM_IRQ);
	x_clear_int(OSTM_IRQ);
}

/*
 *  target_timer.arxmlを使用しない場合の対処
 */
#ifndef SysTimerCnt
#define SysTimerCnt		UINT_C(0)
#endif /* SysTimerCnt */

/*
 *  タイマ割込みハンドラ
 */
ISR(target_timer_hdr)
{
	StatusType ercd;

	/* current_timeを更新する */
	current_time++;

	/*
	 *  カウンタ加算通知処理実行
	 */
	ercd = IncrementCounter(SysTimerCnt);
	/* エラーリターンの場合はシャットダウン */
	if (ercd != E_OK) {
		ShutdownOS(ercd);
	}
}
