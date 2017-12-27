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
 *  $Id: counter.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		カウンタ機能
 */

#ifndef TOPPERS_COUNTER_H
#define TOPPERS_COUNTER_H

#include "queue.h"

/*
 *  カウンタ状態の定義
 *  IncrementCounterのネスト防止策
 */
#define CS_NULL		(FALSE)         /* 非操作中 */
#define CS_DOING	(TRUE)          /* 操作中 */

/*
 *  カウンタIDからカウンタ管理ブロックを取り出すためのマクロ
 */
#define get_cntcb(cntid)	(&(cntcb_table[(cntid)]))

/*
 *  カウンタIDでハードウェアカウンタかチェック用マクロ
 */
#define is_hwcnt(cntid)		((cntid) < tnum_hardcounter)

/*
 *  CNTCBからカウンタIDを取り出すためのマクロ
 */
#define CNTID(p_cntcb)	((CounterType) ((p_cntcb) - cntcb_table))

/*
 *  満了アクションの属性
 */
#define ACTIVATETASK		UINT_C(0x01)    /* タスク起動 */
#define SETEVENT			UINT_C(0x02)    /* イベントセット */
#define CALLBACK			UINT_C(0x04)    /* コールバック */
#define INCREMENTCOUNTER	UINT_C(0x08)    /* カウンタインクリメント */

/*
 *  自動起動の属性
 */
#define ABSOLUTE			UINT_C(0x10)    /* 絶対値起動 */
#define RELATIVE			UINT_C(0x20)    /* 相対値起動 */

/*
 *  各ハードウェアカウンタ処理関数型
 */
typedef void (*HardwareCounterInitRefType)(TickType maxval, TimeType nspertick);    /* 初期化関数型 */
typedef void (*HardwareCounterStartRefType)(void);                                  /* 開始関数型 */
typedef void (*HardwareCounterStopRefType)(void);                                   /* 停止関数型 */
typedef void (*HardwareCounterSetRefType)(TickType exprtick);                       /* 時間設定関数型 */
typedef TickType (*HardwareCounterGetRefType)(void);                                /* 時間取得関数型 */
typedef void (*HardwareCounterCancelRefType)(void);                                 /* 設定時間取消関数型 */
typedef void (*HardwareCounterTriggerRefType)(void);                                /* 強制割込み要求関数型 */
typedef void (*HardwareCounterIntClearRefType)(void);                               /* 割込み要求クリア関数型 */
typedef void (*HardwareCounterIntCancelRefType)(void);                              /* 割込み要求取消関数型 */
typedef void (*HardwareCounterIncrementRefType)(void);                              /* インクリメント関数型 */

/*
 *  ハードウェアカウンタ処理関数型
 */
typedef struct hardware_counter_initialization_block {
	HardwareCounterInitRefType		init;               /* 初期化関数ポインタ */
	HardwareCounterStartRefType		start;              /* 開始関数ポインタ */
	HardwareCounterStopRefType		stop;               /* 停止関数ポインタ */
	HardwareCounterSetRefType		set;                /* 時間設定関数ポインタ */
	HardwareCounterGetRefType		get;                /* 時間取得関数ポインタ*/
	HardwareCounterCancelRefType	cancel;             /* 時間取消関数ポインタ */
	HardwareCounterTriggerRefType	trigger;            /* 強制割込み要求関数ポインタ */
	HardwareCounterIntClearRefType	intclear;           /* 割込み要求クリア関数型 */
	HardwareCounterIntCancelRefType	intcancel;          /* 割込み要求取消関数型 */
	HardwareCounterIncrementRefType	increment;          /* インクリメント関数ポインタ */
	TimeType						nspertick;          /* ハードウェアカウンタでの1ティックの重み(ns単位) */
} HWCNTINIB;

/*
 *  カウンタ初期化ブロック
 */
typedef struct counter_initialization_block {
	TickType	maxval;                                 /* カウンタの最大値 */
	TickType	maxval2;                                /* カウンタの最大値の2倍+1 */
	TickType	tickbase;                               /* OS内部では使用せず，ユーザが自由に使用する値 */
	TickType	mincyc;                                 /* 周期の最小値 */
} CNTINIB;

/*
 *  カウンタ管理ブロック
 */
typedef struct counter_control_block {
	QUEUE			cntexpque;                          /* カウンタ満了キュー */
	const CNTINIB	*p_cntinib;                         /* カウンタ初期化ブロックポインタ */
	TickType		curval;                             /* カウンタの現在ティック */
	boolean			cstat;                              /* カウンタ操作中フラグ */
	boolean			hwset;                              /* ハードウェアカウンタセットフラグ */
} CNTCB;

/*
 *  カウンタ満了情報
 */
typedef struct counter_expire_info CNTEXPINFO;

/*
 *  満了処理関数型
 */
typedef void (*EXPFP)(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb);

/*
 *  カウンタ満了情報
 */
struct counter_expire_info {
	QUEUE		cntexpque;                              /* カウンタ満了キュー(構造体の先頭に入る必要) */
	TickType	expiretick;                             /* 満了するカウンタ上のティック値 */
	EXPFP		expirefunc;                             /* 満了処理関数ポインタ */
};


/*
 *  ハードウェアカウンタ数を保持する変数の宣言（Os_Lcfg.c）
 */
extern const CounterType	tnum_hardcounter;

/*
 *  カウンタ数を保持する変数の宣言（Os_Lcfg.c）
 */
extern const CounterType	tnum_counter;

/*
 *  カウンタ初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const CNTINIB		cntinib_table[];

/*
 *  カウンタ管理ブロックのエリア（Os_Lcfg.c）
 */
extern CNTCB				cntcb_table[];

/*
 *  ハードウェアカウンタ処理関数テーブル（Os_Lcfg.c）
 */
extern const HWCNTINIB		hwcntinib_table[];

/*
 *  ティック値の加算
 */
LOCAL_INLINE TickType
add_tick(TickType val, TickType incr, TickType maxval2)
{
	TickType result;
	/*
	 *  素直な条件式は val + incr <= maxval2 であるが，この条件式で
	 *  は，val + incr が TickType で表せる範囲を超える場合に正しく
	 *  判定できなくなるため，次の条件式としている
	 */
	if (incr <= (maxval2 - val)) {
		result = val + incr;
	}
	else {
		/*
		 *  下の計算式で，val + incr と maxval2 + 1 が TickType で表
		 *  せる範囲を超える場合があるが，オーバフローしても求まる値は
		 *  正しいため差し支えない
		 */
		result = (val + incr) - (maxval2 + 1U);
	}
	return(result);
}

/*
 *  ティック値の差
 */
LOCAL_INLINE TickType
diff_tick(TickType val1, TickType val2, TickType maxval2)
{
	TickType result;

	if (val1 >= val2) {
		result = val1 - val2;
	}
	else {
		/*
		 *  下の計算式で，val1 - val2 と maxval2 + 1 が TickType で表せ
		 *  る範囲を超える場合があるが，オーバフローしても求まる値は正
		 *  しいため差し支えない
		 */
		result = (val1 - val2) + (maxval2 + 1U);
	}
	return(result);
}

/*
 *  カウンタの現在値取得
 *  ソフトウェアカウンタの場合, CNTCBのcurvalデータを返す
 *  ハードウェアカウンタの場合, 最新の現在時間を返す
 */
LOCAL_INLINE TickType
get_curval(const CNTCB *p_cntcb, CounterType cntid)
{
	TickType curval;

	/* カウンタ値の取得 */
	if (is_hwcnt(cntid)) {
		curval = (hwcntinib_table[cntid].get)();
	}
	else {
		curval = p_cntcb->curval;
	}

	return(curval);
}

/*
 *  指定した相対時間からのカウンタ値取得(APIからの取得)
 */
extern TickType get_reltick(const CNTCB *p_cntcb, TickType relval);

/*
 *  指定した絶対時間からのカウンタ値取得(APIからの取得)
 */
extern TickType get_abstick(const CNTCB *p_cntcb, TickType absval);

/*
 *  カウンタ機能の初期化
 */
extern void counter_initialize(void);

/*
 *  カウンタ機能の終了処理
 */
extern void counter_terminate(void);

/*
 *  カウンタ満了キューへの挿入
 */
extern void insert_cnt_expr_que(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb);

/*
 *  カウンタ満了キューから削除
 */
extern void delete_cnt_expr_que(CNTEXPINFO *p_cntexpinfo, CNTCB *p_cntcb);

/*
 *  カウンタの満了処理
 */
extern void expire_process(CNTCB *p_cntcb, CounterType cntid);

/*
 *  ハードウェアカウンタ満了処理
 */
extern void notify_hardware_counter(CounterType cntid);

/*
 *  カウンタのインクリメント
 *
 *  条件：割込み禁止状態で呼ばれる
 */
extern StatusType incr_counter_action(CounterType CounterID);

#endif /* TOPPERS_COUNTER_H */
