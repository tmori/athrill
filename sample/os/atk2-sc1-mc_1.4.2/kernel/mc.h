/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2008-2017 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2017 by FUJI SOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by Spansion LLC, USA
 *  Copyright (C) 2012-2017 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2016 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2014 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2016 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2017 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2017 by Witz Corporation
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
 *  $Id: mc.h 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *		マルチコア管理モジュール
 *		メモリ構造等で変わるマクロ等を定義
 */

#ifndef TOPPERS_MC_H
#define TOPPERS_MC_H

#ifndef TOPPERS_MACRO_ONLY
/*
 *  他コアの CCB へのアクセス
 */
#define get_p_ccb(id)	(p_ccb_table[id])

/*
 *  起動しているOS管理のコア数
 */
extern uint32 activated_cores;

/*
 *  起動時の初期化
 */
extern void ccb_initialize(void);


/*
 *  コア間割込みハンドラ共通処理
 */
extern void ici_handler_main(void);

#ifndef OMIT_KER_REQ_ON_ICI
/*
 *  終了処理要求フラグ
 */
extern volatile boolean	shutdown_reqflg;

#endif /* OMIT_KER_REQ_ON_ICI */

/*
 *  終了処理要求
 */
extern void shutdown_request(void);

/*
 *  バリア同期
 */
extern void barrier_sync(uint8 phase, boolean check_shutdown);

/*
 *		タスク管理モジュール関連
 */

/*
 *  ディスパッチ要求
 */
extern boolean dispatch_request(CCB *p_ccb);


/*
 *		ロック関係機能
 */

#if TTYPE_KLOCK == G_KLOCK

/*
 *  ジャイアントロックの場合
 */

/*
 *  ジャイアントロック変数
 */
extern LockType	giant_lock;

/*
 *  オブジェクトコントロールブロックからロック変数の取り出し
 */
#define GET_SPNLOCK()			(&giant_lock)
#define GET_IOCLOCK()			(&giant_lock)

/*
 *  任意タスク（自タスクも含む）のタスクロックの取得（任意のコンテキスト/1段目）
 */
LOCAL_INLINE void
acquire_tsk_lock(CCB *p_ccb)
{
	x_acquire_lock(&giant_lock);
}

/*
 *  タスクロックの解放（任意のコンテキスト）
 */
LOCAL_INLINE void
release_tsk_lock(CCB *p_ccb)
{
	x_release_lock(&giant_lock);
}

/*
 *  タスクロックの解放とディスパッチ
 */
LOCAL_INLINE void
release_tsk_lock_and_dispatch(CCB *p_ccb, boolean dspreq)
{
	x_release_lock(&giant_lock);
	if (dspreq != FALSE) {
		dispatch();
	}
}

/*
 *  カウンタロックの取得（任意のコンテキスト）
 */
LOCAL_INLINE void
acquire_cnt_lock(CCB *p_ccb)
{
	x_acquire_lock(&giant_lock);
}

/*
 *	カウンタロックの解放（任意のコンテキスト）
 */
LOCAL_INLINE void
release_cnt_lock(CCB *p_ccb)
{
	x_release_lock(&giant_lock);
}

/*
 *	スピンロックの取得（任意のコンテキスト）
 */
LOCAL_INLINE void
acquire_spn_lock(LockType *p_spnlock)
{
	x_acquire_lock(&giant_lock);
}

/*
 *	スピンロックの解放（任意のコンテキスト）
 */
LOCAL_INLINE void
release_spn_lock(LockType *p_spnlock)
{
	x_release_lock(&giant_lock);
}

/*
 *	スピンロックの取得試行（任意のコンテキスト）
 */
LOCAL_INLINE boolean
try_spn_lock(LockType *p_spnlock)
{
	return(x_try_lock(&giant_lock));
}

/*
 *  IOCロックの取得（任意のコンテキスト）
 */
LOCAL_INLINE void
acquire_ioc_lock(LockType *p_ioclock)
{
	x_acquire_lock(&giant_lock);
}

/*
 *	IOCロックの解放（任意のコンテキスト）
 */
LOCAL_INLINE void
release_ioc_lock(LockType *p_ioclock)
{
	x_release_lock(&giant_lock);
}

#else /* TTYPE_KLOCK != G_KLOCK */
/*
 *  コアロックの場合
 */

/*
 *  ロック変数
 */
#if TTYPE_SPN == EMULATE_SPN
extern LockType	spn_lock;
#endif /* TTYPE_SPN == EMULATE_SPN */
extern LockType	ioc_lock;

/*
 *  オブジェクトコントロールブロックからロック変数の取り出し
 */
#if TTYPE_SPN == EMULATE_SPN
#define GET_SPNLOCK()			(&spn_lock)
#endif /* TTYPE_SPN == EMULATE_SPN */
#define GET_IOCLOCK()			(&ioc_lock)

/*
 *  任意タスク（自タスクも含む）のタスクロックの取得（任意のコンテキスト/1段目）
 */
extern void acquire_tsk_lock(CCB *p_ccb);

/*
 *  カウンタロックの取得（任意のコンテキスト）
 */
extern void acquire_cnt_lock(CCB *p_ccb);

/*
 *  タスクロックの解放（任意のコンテキスト）
 */
extern void release_tsk_lock(CCB *p_ccb);

/*
 *  タスクロックの解放とディスパッチ
 */
extern void release_tsk_lock_and_dispatch(CCB *p_ccb,  boolean dspreq);

/*
 * カウンタロックの解放（任意のコンテキスト）
 */
extern void release_cnt_lock(CCB *p_ccb);

/*
 *	スピンロックの取得（任意のコンテキスト）
 */
extern void acquire_spn_lock(LockType *p_spnlock);

/*
 *	スピンロックの解放（任意のコンテキスト）
 */
extern void release_spn_lock(LockType *p_spnlock);

/*
 *	スピンロックの取得試行（任意のコンテキスト）
 */
extern boolean try_spn_lock(LockType *p_spnlock);

/*
 *  IOCロックの取得（任意のコンテキスト）
 */
extern void acquire_ioc_lock(LockType *p_ioclock);

/*
 *	IOCロックの解放（任意のコンテキスト）
 */
extern void release_ioc_lock(LockType *p_ioclock);

#endif /* TTYPE_KLOCK != G_KLOCK */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_MC_H */
