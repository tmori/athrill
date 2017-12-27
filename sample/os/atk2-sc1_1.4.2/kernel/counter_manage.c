/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2011-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2011-2013 by Spansion LLC, USA
 *  Copyright (C) 2011-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2011-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2011-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2011-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2011-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2004-2017 by Witz Corporation
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
 *  $Id: counter_manage.c 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		カウンタ管理モジュール
 */

#include "kernel_impl.h"
#include "check.h"
#include "counter.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_INCCNT_ENTER
#define LOG_INCCNT_ENTER(cntid)
#endif /* LOG_INCCNT_ENTER */

#ifndef LOG_INCCNT_LEAVE
#define LOG_INCCNT_LEAVE(ercd)
#endif /* LOG_INCCNT_LEAVE */

#ifndef LOG_GETCNT_ENTER
#define LOG_GETCNT_ENTER(cntid)
#endif /* LOG_GETCNT_ENTER */

#ifndef LOG_GETCNT_LEAVE
#define LOG_GETCNT_LEAVE(ercd, p_val)
#endif /* LOG_GETCNT_LEAVE */

#ifndef LOG_GETEPS_ENTER
#define LOG_GETEPS_ENTER(cntid, p_val)
#endif /* LOG_GETEPS_ENTER */

#ifndef LOG_GETEPS_LEAVE
#define LOG_GETEPS_LEAVE(ercd, p_val, p_eval)
#endif /* LOG_GETEPS_LEAVE */

/*
 *  カウンタのインクリメント
 */
#ifdef TOPPERS_IncrementCounter

StatusType
IncrementCounter(CounterType CounterID)
{
	StatusType ercd = E_OK;

	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_INCREMENTCOUNTER);
	CHECK_ID(CounterID < tnum_counter);
	CHECK_ID(CounterID >= tnum_hardcounter);

	x_nested_lock_os_int();

	/*
	 *  カウンタのインクリメント
	 *  エラーの場合はincr_counter_actionでエラーフック呼び出しているので，
	 *  ここでは，エラーフックを呼び出さない
	 */
	ercd = incr_counter_action(CounterID);

  d_exit_no_errorhook:
	x_nested_unlock_os_int();
  exit_no_errorhook:
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.cntid = CounterID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_IncrementCounter);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_IncrementCounter */

/*
 *  カウンタ値の参照
 */
#ifdef TOPPERS_GetCounterValue

StatusType
GetCounterValue(CounterType CounterID, TickRefType Value)
{
	StatusType	ercd = E_OK;
	CNTCB		*p_cntcb;

	TickType	curval;

	LOG_GETCNT_ENTER(CounterID);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_GETCOUNTERVALUE);
	CHECK_ID(CounterID < tnum_counter);
	CHECK_PARAM_POINTER(Value);
	p_cntcb = get_cntcb(CounterID);

	/*
	 *  内部処理のため，コンフィギュレーション設定値の２倍＋１までカウント
	 *  アップするのでカウンタ値が設定値よりも大きい場合は設定値を減算する
	 *
	 *  *Value を直接操作してもよいが,局所変数がレジスタに割当てられること
	 *  による速度を期待している
	 */
	x_nested_lock_os_int();
	curval = get_curval(p_cntcb, CounterID);
	x_nested_unlock_os_int();

	if (curval > p_cntcb->p_cntinib->maxval) {
		curval -= (p_cntcb->p_cntinib->maxval + 1U);
	}
	*Value = curval;

  exit_no_errorhook:
	LOG_GETCNT_LEAVE(ercd, Value);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.cntid = CounterID;
	temp_errorhook_par2.p_val = Value;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetCounterValue);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetCounterValue */

/*
 *  経過カウンタ値の参照
 */
#ifdef TOPPERS_GetElapsedValue

StatusType
GetElapsedValue(CounterType CounterID, TickRefType Value, TickRefType ElapsedValue)
{
	StatusType	ercd = E_OK;
	CNTCB		*p_cntcb;

	TickType	curval;

	LOG_GETEPS_ENTER(CounterID, Value);
	CHECK_DISABLEDINT();
	CHECK_CALLEVEL(CALLEVEL_GETELAPSEDVALUE);
	CHECK_ID(CounterID < tnum_counter);
	CHECK_PARAM_POINTER(Value);
	CHECK_PARAM_POINTER(ElapsedValue);
	p_cntcb = get_cntcb(CounterID);

	CHECK_VALUE(*Value <= p_cntcb->p_cntinib->maxval);

	/*
	 *  内部処理のため，コンフィギュレーション設定値の２倍＋１までカウント
	 *  アップするのでカウンタ値が設定値よりも大きい場合は設定値を減算する
	 */
	x_nested_lock_os_int();
	curval = get_curval(p_cntcb, CounterID);
	x_nested_unlock_os_int();

	if (curval > p_cntcb->p_cntinib->maxval) {
		curval -= (p_cntcb->p_cntinib->maxval + 1U);
	}
	*ElapsedValue = diff_tick(curval, *Value, p_cntcb->p_cntinib->maxval);
	*Value = curval;

  exit_no_errorhook:
	LOG_GETEPS_LEAVE(ercd, Value, ElapsedValue);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  exit_errorhook:
	x_nested_lock_os_int();
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.cntid = CounterID;
	temp_errorhook_par2.p_val = Value;
	temp_errorhook_par3.p_eval = ElapsedValue;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_GetElapsedValue);
	x_nested_unlock_os_int();
	goto exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_GetElapsedValue */

/*
 *  ハードウェアカウンタ満了処理
 *
 *  割込みルーチンより実行される
 */
#ifdef TOPPERS_notify_hardware_counter

void
notify_hardware_counter(CounterType cntid)
{
	CNTCB *p_cntcb;

	p_cntcb = get_cntcb(cntid);

	/* カウンタ満了処理中はOS割込みを禁止 */
	x_nested_lock_os_int();

	/*
	 *  ハードウェアカウンタに対応するC2ISRが起動した際に，
	 *  割込み要求のクリア処理を実行する
	 */
	(hwcntinib_table[cntid].intclear)();

	expire_process(p_cntcb, cntid);

	x_nested_unlock_os_int();
}

#endif /* TOPPERS_notify_hardware_counter */

/*
 *  カウンタのインクリメント
 *
 *  条件：割込み禁止状態で呼ばれる
 */
#ifdef TOPPERS_incr_counter_action

StatusType
incr_counter_action(CounterType CounterID)
{
	StatusType	ercd = E_OK;
	TickType	newval;
	CNTCB		*p_cntcb;

	LOG_INCCNT_ENTER(CounterID);
	p_cntcb = get_cntcb(CounterID);

	/*
	 *  カウンタが操作中(IncrementCounterのネスト）の場合エラー
	 *  ※独自仕様
	 */
	D_CHECK_STATE(p_cntcb->cstat == CS_NULL);

	p_cntcb->cstat = CS_DOING;

	newval = add_tick(p_cntcb->curval, 1U, p_cntcb->p_cntinib->maxval2);

	p_cntcb->curval = newval;

	expire_process(p_cntcb, CounterID);

	p_cntcb->cstat = CS_NULL;

  d_exit_no_errorhook:
	LOG_INCCNT_LEAVE(ercd);
	return(ercd);

#ifdef CFG_USE_ERRORHOOK
  d_exit_errorhook:
#ifdef CFG_USE_PARAMETERACCESS
	temp_errorhook_par1.cntid = CounterID;
#endif /* CFG_USE_PARAMETERACCESS */
	call_errorhook(ercd, OSServiceId_IncrementCounter);
	goto d_exit_no_errorhook;
#endif /* CFG_USE_ERRORHOOK */
}

#endif /* TOPPERS_incr_counter_action */
