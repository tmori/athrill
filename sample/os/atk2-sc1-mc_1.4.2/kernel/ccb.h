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
 *  $Id: ccb.h 2401 2017-03-14 09:09:24Z witz-itoyo $
 */

/*
 *  コア管理ブロック
 */
#ifndef TOPPERS_CCB_H
#define TOPPERS_CCB_H

#include "target_ccb.h"

#ifndef TOPPERS_MACRO_ONLY

#include "queue.h"
#include "spinlock.h"

/*
 *  前方参照
 */
typedef struct isr_control_block					ISRCB;
typedef struct os_application_initialization_block	OSAPINIB;

/*
 *		コア管理ブロック
 */
struct core_control_block {

	CoreIdType coreid;

	/*
	 *  カーネル動作状態フラグ
	 */
	boolean kerflg;

	/*
	 *  SuspendAllInterrupts のネスト回数
	 */
	uint8 sus_all_cnt;

	/*
	 *  SuspendOSInterrupts のネスト回数
	 */
	uint8 sus_os_cnt;

	/*
	 *  OS実行制御のための変数
	 */
	uint16 callevel_stat;       /* 実行中のコンテキスト */

#if TTYPE_KLOCK == C_KLOCK
	/*
	 *  コアロック変数
	 */
	LockType	tsk_lock;
	LockType	cnt_lock;
#endif /* TTYPE_KLOCK == C_KLOCK */

	/*
	 *		タスク関連
	 */

	/*
	 *  実行状態のタスク
	 *
	 *  実行状態のタスク（＝プロセッサがコンテキストを持っているタスク）の
	 *  TCBを指すポインタ．実行状態のタスクがない場合はNULLにする．
	 *
	 *  サービスコールの処理中で，自タスク（サービスコールを呼び出したタス
	 *  ク）に関する情報を参照する場合はp_runtskを使う．p_runtskを書き換え
	 *  るのは，ディスパッチャ（と初期化処理）のみである．
	 */
	TCB *p_runtsk;

	/*
	 *  最高優先順位のタスク
	 *
	 *  実行できるタスクの中で最高優先順位のタスクのTCBを指すポインタ．実
	 *  行できるタスクがない場合はNULLにする．
	 *
	 */
	TCB *p_schedtsk;

	/*
	 *  レディキュー中の最高優先度
	 *  レディキューには実行可能状態のタスクのみを含むので，実行可能状態の
	 *  タスクの中での最高優先度を保持する．レディキューが空の時（実行可能
	 *  状態のタスクが無い時）は TPRI_MINTASKにする．
	 */
	PriorityType nextpri;

	/*
	 *  レディキュー
	 *
	 *  レディキューは，実行できる状態のタスクを管理するためのキューである．
	 *  実行状態のタスクも管理しているため，レディ（実行可能）キューという
	 *  名称は正確ではないが，レディキューという名称が定着しているため，こ
	 *  の名称で呼ぶことにする．
	 *
	 *  レディキューは，優先度ごとのタスクキューで構成されている．タスクの
	 *  TCBは，該当する優先度のキューに登録される．
	 */
	QUEUE ready_queue[TNUM_TPRI];

	/*
	 *  レディキューサーチのためのビットマップ
	 *
	 *  レディキューのサーチを効率よく行うために，優先度ごとのタスクキュー
	 *  にタスクが入っているかどうかを示すビットマップを用意している．ビッ
	 *  トマップを使うことで，メモリアクセスの回数を減らすことができるが，
	 *  ビット操作命令が充実していないプロセッサで，優先度の段階数が少ない
	 *  場合には，ビットマップ操作のオーバーヘッドのために，逆に効率が落ち
	 *  る可能性もある．
	 */
	uint16 ready_primap;

	/*
	 *		割込み関連
	 */
	/*
	 *  実行中のC2ISR
	 *
	 *  C2ISRを実行していない時は，ISRID_NULL にする．
	 */
	ISRCB *p_runisr;

	/*
	 *  wrap_sus_all_int のネスト回数
	 */
	uint8 wrap_sus_all_cnt;

	/*
	 *  wrap_sus_os_int のネスト回数
	 */
	uint8 wrap_sus_os_cnt;

	/*
	 *  エラーフックに渡す情報を格納する変数
	 */
	OSServiceIdType	temp_errorhook_svcid;
	ErrorHook_Par	temp_errorhook_par1;
	ErrorHook_Par	temp_errorhook_par2;
	ErrorHook_Par	temp_errorhook_par3;
	ErrorHook_Par	errorhook_par1;
	ErrorHook_Par	errorhook_par2;
	ErrorHook_Par	errorhook_par3;

	/*
	 *  ユーザ定義コア間割込み要求ビットマップ
	 */
	uint32 ici_request_map;

	/*
	 *  コア間割込み許可ビットマスク
	 */
	uint32 ici_bit_mask;

	/*
	 *  ディスパッチ用コア間割込み要求フラグ
	 */
	boolean ici_disreqflg;

	/*
	 *  実行中のOSアプリケーション
	 */
	const OSAPINIB *p_currentosap;

	/*
	 *  ターゲット依存 コアコントロールブロック
	 */
	TCCB target_ccb;

	/*
	 * プロテクションフック用スピンロック管理ブロックポインタ
	 */
	SPNCB *p_protectspncb;
};

/*
 *  CCBへのアクセステーブル（Os_Lcfg.c）
 */
extern CCB * const p_ccb_table[];


#ifndef OMIT_GET_MY_P_CCB
/*
 *  自コアの CCBアドレス取得関数
 */
LOCAL_INLINE CCB *
get_my_p_ccb(void)
{
	return(p_ccb_table[x_core_id()]);
}
#endif /* OMIT_GET_MY_P_CCB */

#endif /* TOPPERS_MACRO_ONLY */

#endif /* TOPPERS_CCB_H */
