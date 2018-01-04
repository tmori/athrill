/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2014 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: target_hw_counter.c 546 2015-12-29 01:11:53Z ertl-honda $
 */

/*
 *		ハードウェアカウンタのターゲット依存定義（fl850f1l用）
 */
#include "Os.h"
#include "prc_sil.h"
#include "target_hw_counter.h"
#include "tauj_hw_counter.h"

/* カウンタの最大値の2倍+1 */
static TickType MAIN_HW_COUNTER_maxval[TNUM_HWCORE];

/*
 *  =begin MAIN_HW_COUNTER_CORE0の定義
 */

/*
 *  ハードウェアカウンタの初期化
 */
void
init_hwcounter_MAIN_HW_COUNTER_CORE0(TickType maxval, TimeType nspertick)
{
	init_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID,
						HWC_CTIM_CORE0_UNIT, HWC_CTIM_CORE0_ID,
						maxval, nspertick, &MAIN_HW_COUNTER_maxval[0]);
}

/*
 *  ハードウェアカウンタの開始
 */
void
start_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	start_hwcounter_tauj(HWC_CTIM_CORE0_UNIT, HWC_CTIM_CORE0_ID);
}

/*
 *  ハードウェアカウンタの停止
 */
void
stop_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	stop_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID,
						HWC_CTIM_CORE0_UNIT, HWC_CTIM_CORE0_ID);
}

/*
 *  ハードウェアカウンタへの満了時間の設定
 */
void
set_hwcounter_MAIN_HW_COUNTER_CORE0(TickType exprtick)
{
	set_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID,
					   HWC_CTIM_CORE0_UNIT, HWC_CTIM_CORE0_ID,
					   exprtick, MAIN_HW_COUNTER_maxval[0]);
}

/*
 *  ハードウェアカウンタの現在時間の取得
 */
TickType
get_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	return(get_hwcounter_tauj(HWC_CTIM_CORE0_UNIT, HWC_CTIM_CORE0_ID,
							  MAIN_HW_COUNTER_maxval[0]));
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
cancel_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	cancel_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID);
}

/*
 *  ハードウェアカウンタの強制割込み要求
 */
void
trigger_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	trigger_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID);
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
int_clear_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	int_clear_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID);
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
int_cancel_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	int_cancel_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID);
}

/*
 *  ハードウェアカウンタのインクリメント
 */
void
increment_hwcounter_MAIN_HW_COUNTER_CORE0(void)
{
	increment_hwcounter_tauj(HWC_DTIM_CORE0_UNIT, HWC_DTIM_CORE0_ID);
}

/*
 *  =end MAIN_HW_COUNTER_CORE0の定義
 */


/*
 *  =begin MAIN_HW_COUNTER_CORE1の定義
 */

/*
 *  ハードウェアカウンタの初期化
 */
void
init_hwcounter_MAIN_HW_COUNTER_CORE1(TickType maxval, TimeType nspertick)
{
	init_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID,
						HWC_CTIM_CORE1_UNIT, HWC_CTIM_CORE1_ID,
						maxval, nspertick, &MAIN_HW_COUNTER_maxval[1]);
}

/*
 *  ハードウェアカウンタの開始
 */
void
start_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	start_hwcounter_tauj(HWC_CTIM_CORE1_UNIT, HWC_CTIM_CORE1_ID);
}

/*
 *  ハードウェアカウンタの停止
 */
void
stop_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	stop_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID,
						HWC_CTIM_CORE1_UNIT, HWC_CTIM_CORE1_ID);
}

/*
 *  ハードウェアカウンタへの満了時間の設定
 */
void
set_hwcounter_MAIN_HW_COUNTER_CORE1(TickType exprtick)
{
	set_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID,
					   HWC_CTIM_CORE1_UNIT, HWC_CTIM_CORE1_ID,
					   exprtick, MAIN_HW_COUNTER_maxval[1]);
}

/*
 *  ハードウェアカウンタの現在時間の取得
 */
TickType
get_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	return(get_hwcounter_tauj(HWC_CTIM_CORE1_UNIT, HWC_CTIM_CORE1_ID,
							  MAIN_HW_COUNTER_maxval[1]));
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
cancel_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	cancel_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID);
}

/*
 *  ハードウェアカウンタの強制割込み要求
 */
void
trigger_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	trigger_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID);
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
int_clear_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	int_clear_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID);
}

/*
 *  ハードウェアカウンタの設定された満了時間の取消
 */
void
int_cancel_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	int_cancel_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID);
}

/*
 *  ハードウェアカウンタのインクリメント
 */
void
increment_hwcounter_MAIN_HW_COUNTER_CORE1(void)
{
	increment_hwcounter_tauj(HWC_DTIM_CORE1_UNIT, HWC_DTIM_CORE1_ID);
}

/*
 *  =end MAIN_HW_COUNTER_CORE1の定義
 */
