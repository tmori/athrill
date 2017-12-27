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
 *  @(#) $Id: ostm.h 164 2015-06-03 01:22:29Z t_ishikawa $
 */

#ifndef TOPPERS_OSTM_H
#define TOPPERS_OSTM_H

#include "prc_sil.h"

#ifndef TOPPERS_MACRO_ONLY

/*
 *  タイマ値の内部表現とマイクロ秒単位との変換
 *
 */
#define TO_USEC(clock)	((((SystemTimeUsType) (clock)) * 1000U) / TIMER_CLOCK)

/*
 *  タイマ値の内部表現と100ナノ秒単位との変換
 *
 */
#define TO_100NSEC(clock)	((((SystemTimeUsType) (clock)) * 10000U) / TIMER_CLOCK)

/*
 *  マイクロ秒単位での時刻を取得
 */
extern SystemTimeUsType get_tim_utime(void);

/*
 *  100ナノ秒単位での時刻を取得
 */
extern SystemTime100NsType get_tim_100ntime(void);

/*
 *  タイマの起動処理
 *
 *  タイマを初期化し，周期的なタイマ割込み要求を発生させる
 */
extern void target_timer_initialize(void);

/*
 *  タイマの停止処理
 *
 *  タイマの動作を停止させる
 */
extern void target_timer_terminate(void);

/*
 *  タイマの現在値の読出し
 */
LOCAL_INLINE TickType
target_timer_get_current(void)
{
	TickType count;

	count = sil_rew_mem((void *) (OSTM_CNT_W));

	return(TIMER_CLOCK - count);
}

/*
 *  タイマ割込み要求のチェック
 */
LOCAL_INLINE boolean
target_timer_probe_int(void)
{
	return(x_probe_int(OSTM_IRQ));
}

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_OSTM_H */
