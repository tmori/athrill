/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2011-2017 by Witz Corporation
 *  Copyright (C) 2014-2016 by AISIN COMCRUISE Co., Ltd., JAPAN
 *  Copyright (C) 2014-2016 by eSOL Co.,Ltd., JAPAN
 *  Copyright (C) 2014-2017 by SCSK Corporation, JAPAN
 *  Copyright (C) 2015-2017 by SUZUKI MOTOR CORPORATION
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
 *  $Id: trace_config.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		トレースログ機能
 */

#include "kernel_impl.h"
#include "task.h"

#include "target_timer.h"

/*
 *  内部関数プロトタイプ宣言
 */
static StatusType trace_wri_log(TRACE *p_trace);

/*
 *  トレースログバッファとそれにアクセスするためのポインタ
 */
static TRACE			trace_buffer[TCNT_TRACE_BUFFER];    /* トレースログバッファ */
static uint32			trace_count;                        /* トレースログバッファ中のログの数 */
static uint32			trace_head;                         /* 先頭のトレースログの格納位置 */
static uint32			trace_tail;                         /* 次のトレースログの格納位置 */
static TraceModeType	trace_mode = TRACE_AUTOSTOP;        /* トレースモード */

/*
 *  トレースログ機能の初期化
 */
void
trace_initialize(uintptr exinf)
{
	TraceModeType mode = ((TraceModeType) exinf);

	trace_count = 0U;
	trace_head = 0U;
	trace_tail = 0U;
	trace_mode = mode;
}

/*
 *  トレースログの開始
 */
StatusType
trace_sta_log(TraceModeType mode)
{
	if ((mode & TRACE_CLEAR) != 0U) {
		trace_count = 0U;
		trace_head = 0U;
		trace_tail = 0U;
	}
	trace_mode = mode;
	return(E_OK);
}

/*
 *  トレースログの書込み
 */
static StatusType
trace_wri_log(TRACE *p_trace)
{
	SIL_PRE_LOC;

	if (trace_mode != TRACE_STOP) {
		SIL_LOC_INT();

		/*
		 *  トレース時刻の設定
		 *
		 *  LOG_WRI_LOG_ENTERから呼ばれた場合にp_trace->logtimを書き換
		 *  えてしまうのは気持ちが悪いが，wri_logの方で上書きするため問
		 *  題はない
		 */
		p_trace->logtim = GET_UTIM();

		/*
		 *  トレースバッファに記録
		 */
		trace_buffer[trace_tail] = *p_trace;
		trace_tail++;
		if (trace_tail >= TCNT_TRACE_BUFFER) {
			trace_tail = 0U;
		}
		if (trace_count < TCNT_TRACE_BUFFER) {
			trace_count++;
			if ((trace_count >= TCNT_TRACE_BUFFER)
				&& ((trace_mode & TRACE_AUTOSTOP) != 0U)) {
				trace_mode = TRACE_STOP;
			}
		}
		else {
			trace_head = trace_tail;
		}

		SIL_UNL_INT();
	}
	return(E_OK);
}

/*
 *  トレースログの読出し
 */
StatusType
trace_rea_log(TRACE *p_trace)
{
	StatusType ercd;
	SIL_PRE_LOC;

	SIL_LOC_INT();

	/*
	 *  トレースログバッファからの取出し
	 */
	if (trace_count > 0U) {
		*p_trace = trace_buffer[trace_head];
		trace_count--;
		trace_head++;
		if (trace_head >= TCNT_TRACE_BUFFER) {
			trace_head = 0U;
		}
		ercd = E_OK;
	}
	else {
		ercd = E_NOT_OK;
	}

	SIL_UNL_INT();
	return(ercd);
}

/*
 *  トレースログを出力するためのライブラリ関数
 */

void
trace_write_0(uint32 type)
{
	TRACE trace;

	trace.logtype = type;
	(void) trace_wri_log(&trace);
}

void
trace_write_1(uint32 type, const uintptr arg1)
{
	TRACE trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	(void) trace_wri_log(&trace);
}

void
trace_write_2(uint32 type, uintptr arg1, uintptr arg2)
{
	TRACE trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	trace.loginfo[1] = arg2;
	(void) trace_wri_log(&trace);
}

void
trace_write_3(uint32 type, uintptr arg1, uintptr arg2, uintptr arg3)
{
	TRACE trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	trace.loginfo[1] = arg2;
	trace.loginfo[2] = arg3;
	(void) trace_wri_log(&trace);
}

void
trace_write_4(uint32 type, uintptr arg1, uintptr arg2, uintptr arg3,
			  uintptr arg4)
{
	TRACE trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	trace.loginfo[1] = arg2;
	trace.loginfo[2] = arg3;
	trace.loginfo[3] = arg4;
	(void) trace_wri_log(&trace);
}

void
trace_write_5(uint32 type, uintptr arg1, uintptr arg2, uintptr arg3,
			  uintptr arg4, uintptr arg5)
{
	TRACE trace;

	trace.logtype = type;
	trace.loginfo[0] = arg1;
	trace.loginfo[1] = arg2;
	trace.loginfo[2] = arg3;
	trace.loginfo[3] = arg4;
	trace.loginfo[4] = arg5;
	(void) trace_wri_log(&trace);
}

/*
 *  アセンブリ言語で記述されるコードからトレースログを出力するための関
 *  数
 */
void
log_dsp_enter(const TCB *p_tcb)
{
	trace_1(LOG_TYPE_DSP | LOG_ENTER, (const uintptr) p_tcb);
}

void
log_dsp_leave(const TCB *p_tcb)
{
	trace_1(LOG_TYPE_DSP | LOG_LEAVE, (const uintptr) p_tcb);
}
