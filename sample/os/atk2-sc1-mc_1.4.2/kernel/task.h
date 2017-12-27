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
 *  $Id: task.h 727 2017-01-23 09:27:59Z witz-itoyo $
 */

/*
 *		タスク管理機能
 */

#ifndef TOPPERS_TASK_H
#define TOPPERS_TASK_H

#include "queue.h"
#include "resource.h"
#include "mc.h"
#include "osap.h"
#include "spinlock.h"

/*
 *  イベントマスク値の定義
 */
#define EVTMASK_NONE	((EventMaskType) 0) /* イベントなし */

/*
 *  優先度値の定義（内部表現）
 */
#define TPRI_MINTASK	((PriorityType) (TNUM_TPRI - 1U))               /* 最低タスク優先度 */
#define TPRI_MAXTASK	((PriorityType) (0))                            /* 最高タスク優先度 */

/*
 *  タスクIDからTCBを取り出すためのマクロ
 */
#define get_tcb(tskid)		(p_tcb_table[(tskid)])

/*
 *  TCBからタスクIDを取り出すためのマクロ
 *  p_tcb がNULLの場合は使えない
 */
#define TSKID(p_tcb)	((TaskType) (((p_tcb)->p_tinib) - tinib_table))

/*
 *  タスク数を保持する変数の宣言（Os_Lcfg.c）
 */
extern const TaskType	tnum_task;          /* タスクの数 */
extern const TaskType	tnum_exttask;       /* 拡張タスクの数 */


/*
 *  タスク初期化ブロック
 *
 *  タスクに関する情報を，値が変わらないためにROMに置ける部分（タスク
 *  初期化ブロック）と，値が変化するためにRAMに置かなければならない部
 *  分（タスク管理ブロック，TCB）に分離し，TCB内に対応するタスク初期化
 *  ブロックを指すポインタを入れる
 *  タスク初期化ブロック内に対応するTCBを指すポインタを入れる方法の方が，
 *  RAMの節約の観点からは望ましいが，実行効率が悪くなるために採用
 *  していない
 *  他のオブジェクトについても同様に扱う
 */
typedef struct task_initialization_block {
	FunctionRefType	task;       /* タスクの起動番地 */

#ifdef USE_TSKINICTXB
	TSKINICTXB tskinictxb;           /* タスク初期化コンテキストブロック */
#else /* USE_TSKINICTXB */
	MemorySizeType	stksz;              /* スタック領域のサイズ（丸めた値） */
	void			*stk;               /* スタック領域の先頭番地 */
#endif /* USE_TSKINICTXB */
	const OSAPINIB	*p_osapinib;    /* 所属するOSアプリケーションの管理ブロック */
	PriorityType	inipri;         /* 初期優先度 （内部表現）*/
	PriorityType	exepri;         /* 実行開始時の優先度 （内部表現）*/
	uint8			maxact;         /* 多重起動要求の最大数 */
	AppModeType		autoact;        /* 起動するモード */
	CCB				*p_ccb;         /* 所属するコア管理ブロックへのポインタ */
} TINIB;

/*
 *  タスク管理ブロック（Os_Lcfg.c）
 */
struct task_control_block {
	QUEUE			task_queue;     /* タスクキュー(構造体の先頭に入る必要) */
	const TINIB		*p_tinib;       /* 初期化ブロックへのポインタ */
	PriorityType	curpri;         /* 現在の優先度（内部表現）*/
	TaskStateType	tstat;          /* タスク状態（内部表現）*/
	uint8			actcnt;         /* 多重起動要求数 */
	EventMaskType	curevt;         /* イベントの現在値 */
	EventMaskType	waievt;         /* 待っているイベント */
	RESCB			*p_lastrescb;   /* 最後に獲得したリソース管理ブロックへのポインタ */
	SPNCB			*p_lastspncb;   /* 最後に取得したスピンロック管理ブロックへのポインタ */
	TSKCTXB			tskctxb;        /* タスクコンテキストブロック */
};

/*
 *  タスク初期化ブロックのエリア（Os_Lcfg.c）
 */
extern const TINIB	tinib_table[];

/*
 *  TCBのエリア（Os_Lcfg.c）
 */
extern TCB * const	p_tcb_table[];

/*
 *  タスク管理モジュールの初期化
 */
extern void task_initialize(void);

/*
 *  タスクの起動
 *
 *  対象タスク（p_tcbで指定したタスク）を起動する
 *  （休止状態から実行できる状態に遷移させる）
 *  タスクの起動時に必要な初期化を行う
 */
extern boolean make_active(TCB *p_tcb);

/*
 *  実行できる状態への移行
 *
 *  対象タスク（p_tcbで指定したタスク）を実行できる状態に遷移させる
 *  対象タスクの優先度が，最高優先度タスク（schedtsk）の優先度よりも高
 *  い場合には，対象タスクを新しい最高優先度タスクとし，それまでの最高
 *  優先度タスクをレディキューの先頭に入れる
 *  そうでない場合には，対象タスクをレディキューの末尾に入れる
 *  対象タスクを最高優先度タスクとした場合に，TRUE を返す
 */
extern boolean make_runnable(TCB *p_tcb);

/*
 *  実行できる状態から他の状態への遷移
 */
extern void make_non_runnable(CCB *p_ccb);

/*
 *  最高優先順位タスクのサーチ
 *
 *  レディキュー中の最高優先順位のタスクをサーチする
 *  レディキューが空の場合には，この関数を呼び出してはならない
 */
extern void search_schedtsk(CCB *p_ccb);

/*
 *  タスクのプリエンプト
 *
 *  自タスクを実行可能状態に移行させ，最高優先度タスクを実行状態にする
 *  この関数から戻った後に，dispatch を呼び出して他のタスクへ切り替える
 *  ことを想定している
 */
extern void preempt(CCB *p_ccb, PriorityType pre_pri);

/*
 *  実行中のタスクをSUSPENDED状態にする
 */
extern void suspend(CCB *p_ccb);

/*
 *  満了処理専用タスクの起動
 *
 *  条件：OS割込み禁止状態で呼ばれる
 */
extern StatusType activate_task_action(TaskType TaskID);

/*
 *  満了処理専用イベントのセット
 *
 *  条件：OS割込み禁止状態で呼ばれる
 */
extern StatusType set_event_action(TaskType TaskID, EventMaskType Mask);

/*
 *  タスク不正終了時に呼ぶ関数
 */
extern void exit_task(void);

#endif /* TOPPERS_TASK_H_ */
