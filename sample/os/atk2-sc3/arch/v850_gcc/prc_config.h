/*
 *  TOPPERS ATK2
 *      Toyohashi Open Platform for Embedded Real-Time Systems
 *      Automotive Kernel Version 2
 *
 *  Copyright (C) 2012-2013 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2012-2013 by FUJISOFT INCORPORATED, JAPAN
 *  Copyright (C) 2012-2013 by FUJITSU VLSI LIMITED, JAPAN
 *  Copyright (C) 2012-2013 by NEC Communication Systems, Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Panasonic Advanced Technology Development Co., Ltd., JAPAN
 *  Copyright (C) 2012-2013 by Renesas Electronics Corporation, JAPAN
 *  Copyright (C) 2012-2013 by Sunny Giken Inc., JAPAN
 *  Copyright (C) 2012-2013 by TOSHIBA CORPORATION, JAPAN
 *  Copyright (C) 2012-2013 by Witz Corporation, JAPAN
 *  Copyright (C) 2013 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: prc_config.h 189 2015-06-26 01:54:57Z t_ishikawa $
 */

/*
 *		プロセッサ依存モジュール（V850用）
 *
 *  このインクルードファイルは，target_config.h（または，そこからインク
 *  ルードされるファイル）のみからインクルードされる．他のファイルから
 *  直接インクルードしてはならない
 */

#ifndef TOPPERS_PRC_CONFIG_H
#define TOPPERS_PRC_CONFIG_H

#include "prc_sil.h"

/*
 *  エラーチェック方法の指定
 */
/* =begin modified for SC3 */
#ifndef CHECK_SSTKSZ_ALIGN
#define CHECK_SSTKSZ_ALIGN	UINT_C(4)   /* システムスタックサイズのアライン単位 */
#endif /* CHECK_STKSZ_ALIGN */

#ifndef CHECK_USTKSZ_ALIGN
#define CHECK_USTKSZ_ALIGN	UINT_C(16)   /* ユーザスタックサイズのアライン単位 */
#endif /* CHECK_USTKSZ_ALIGN */
/* =end modified for SC3 */

#ifndef CHECK_FUNC_ALIGN
#define CHECK_FUNC_ALIGN	UINT_C(2)   /* 関数のアライン単位 */
#endif /* CHECK_FUNC_ALIGN */

#define CHECK_FUNC_NONNULL      /* 関数の非NULLチェック */

#ifndef CHECK_STACK_ALIGN
#define CHECK_STACK_ALIGN	UINT_C(4)   /* スタック領域のアライン単位 */
#endif /* CHECK_STACK_ALIGN */

#define CHECK_STACK_NONNULL     /* スタック領域の非NULLチェック */

/* =begin modified for SC3 */
/*
 *  ターゲットで追加するシステムサービステーブル定義
 */
#ifndef OTHER_PRC_SVC_TABLE
#ifdef USE_KERNEL_LIBRARY_SYSLOG
/* シスログ機能有効時 */
#define PRC_SVC_TABLE	(void *) &_kernel_KernelLibrarySyslog
#define PRC_SVC_NUM		1
#else /* USE_KERNEL_LIBRARY_SYSLOG */
/* シスログ機能無効時 */
#define PRC_SVC_TABLE
#define PRC_SVC_NUM		0
#endif /* USE_KERNEL_LIBRARY_SYSLOG */
#else /* OTHER_PRC_SVC_TABLE */
#define STR(a)	STR_(a)
#define STR_(a)	# a
#include STR(OTHER_PRC_SVC_TABLE)
#endif /* OTHER_PRC_SVC_TABLE */

/*
 *  svcで呼び出す処理の種類
 *   システムサービスの入り口
 *   非信頼フックの出口
 */
#define SYSCALL_SIZE    2
/* =end modified for SC3 */

#define call_errorhook(ercd, svcid)		stack_change_and_call_func_2(&internal_call_errorhook, (ercd), (svcid))
#define call_shutdownhook(ercd)			stack_change_and_call_func_1(&internal_call_shtdwnhk, ((uint32) (ercd)))

/*
 *  割込み番号の範囲の判定
 */
#if TMIN_INTNO == 0
#define VALID_INTNO(intno)	((intno) <= (InterruptNumberType) (TMAX_INTNO))
#else /* TMIN_INTNO != 0 */
#define VALID_INTNO(intno)	(((InterruptNumberType) (TMIN_INTNO) <= (intno)) && ((intno) <= (InterruptNumberType) (TMAX_INTNO)))
#endif /* TMIN_INTNO == 0 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  非タスクコンテキスト用のスタック開始アドレス設定マクロ
 */
#define TOPPERS_OSTKPT(stk, stksz)	((StackType *) ((sint8 *) (stk) + (stksz)))

/*
 *  プロセッサの特殊命令のインライン関数定義
 */
#include <prc_insn.h>

/*
 *  TOPPERS標準割込み処理モデルの実現
 */

/*
 * 例外（割込み/CPU例外）のネスト回数のカウント
 *
 * コンテキスト参照のために使用
 */
extern uint32		except_nest_cnt;

/*
 *  カーネル管理内（カテゴリ2）の割込みを禁止するためのPMRの設定値
 */
extern const uint16	pmr_isr2_mask;

/*
 *  カーネル管理外（カテゴリ1）の割込みとなる優先度のPMRのビット
 */
extern const uint16	pmr_isr1_mask;

/*
 *  現在の割込み優先度の値（内部表現）
 */
extern uint8		current_iintpri;

/*
 * 割り込み要求マスクテーブル（PMRへの設定値）
 */
extern const uint16	pmr_setting_tbl[];

/*
 *  無限ループ処理
 */
extern void infinite_loop(void) NoReturn;

/*
 *  割込み優先度の内部表現と外部表現の変更
 */
#ifndef TOPPERS_MACRO_ONLY
#define EXT_IPM(iipm)	((PriorityType) (iipm - (TNUM_INTPRI)))     /* 内部表現を外部表現に */
#define INT_IPM(ipm)	((uint32) (ipm + (TNUM_INTPRI)))            /* 外部表現を内部表現に */
#else /* TOPPERS_MACRO_ONLY */
#define EXT_IPM(iipm)	(iipm - (TNUM_INTPRI))              /* 内部表現を外部表現に */
#define INT_IPM(ipm)	(ipm + (TNUM_INTPRI))               /* 外部表現を内部表現に */
#endif /* TOPPERS_MACRO_ONLY */

/*
 *  全割込み禁止状態への移行
 */
LOCAL_INLINE void
x_lock_all_int(void)
{
	disable_int();
}

/*
 *  全割込み禁止状態の解除
 */
LOCAL_INLINE void
x_unlock_all_int(void)
{
	enable_int();
}

/*
 *  x_nested_lock_os_int()のネスト回数
 *  アクセスの順序が変化しないよう，volatile を指定
 */
extern volatile uint8 nested_lock_os_int_cnt;

/*
 *  OS割込み禁止
 *  API実行時に呼び出される
 *  割込み優先度マスクがISR2の優先度マスクの範囲より高い状態で呼び出される
 *  ことはない(ISR1から呼び出されることはない)
 */
LOCAL_INLINE void
x_nested_lock_os_int(void)
{
	/*
	 *  一段目なら割込みの禁止レベルに設定割込み優先度マスクの値を保存
	 */
	if (nested_lock_os_int_cnt == 0U) {
		set_pmr(pmr_isr2_mask);   /* OS割り込み禁止に設定 */
	}
	nested_lock_os_int_cnt++;

	V850_MEMORY_CHANGED
}

/*
 *  OS割込み解除
 *  API実行時に呼び出される
 *  割込み優先度マスクがISR2の優先度マスクの範囲より高い状態で呼び出される
 *  ことはない(ISR1から呼び出されることはない)
 */
LOCAL_INLINE void
x_nested_unlock_os_int(void)
{
	V850_MEMORY_CHANGED
	ASSERT(nested_lock_os_int_cnt > 0U);
	nested_lock_os_int_cnt--;

	/*
	 *  一番外側なら割込み優先度マスクを更新
	 */
	if (nested_lock_os_int_cnt == 0U) {
		set_pmr(pmr_setting_tbl[current_iintpri]);
	}
}

/*
 *  SuspendOSInterrupts()から呼び出すOS割込み禁止状態への移行
 *  x_nested_lock_os_int()との違いは，ISR1から呼び出される可能性があるため，
 *  現在のISPRがOS割込み禁止時の値より小さい場合はISR1から呼び出されたとして，
 *  何もせずリターンする
 */
LOCAL_INLINE void
x_suspend_lock_os_int(void)
{
	uint16 ispr;

	ispr = get_ispr();

	if ((ispr & pmr_isr1_mask) == 0U) {
		/*
		 *  カテゴリ1 ISRから呼ばれた場合は何もしない
		 */
		if (nested_lock_os_int_cnt == 0U) {
			set_pmr(pmr_isr2_mask);   /* OS割り込み禁止に設定 */
		}
		nested_lock_os_int_cnt++;
		V850_MEMORY_CHANGED
	}
}

/*
 *  ResumeOSInterrupts()から呼び出すOS割込み禁止状態の解除
 *  x_nested_lock_os_int()との違いは，ISR1から呼び出される可能性があるため，
 *  現在のISPRがOS割込み禁止時の値より小さい場合はISR1から呼び出されたとして，
 *  何もせずリターンする
 */
LOCAL_INLINE void
x_resume_unlock_os_int(void)
{
	uint16	ispr;

	V850_MEMORY_CHANGED
	ispr = get_ispr();

	if ((ispr & pmr_isr1_mask) == 0U) {
		/*
		 *  カテゴリ1 ISRから呼ばれた場合は何もしない
		 */
		ASSERT(nested_lock_os_int_cnt > 0U);
		nested_lock_os_int_cnt--;

		/*
		 *  一番外側なら割込み優先度マスクを更新
		 */
		if (nested_lock_os_int_cnt == 0U) {
			set_pmr(pmr_setting_tbl[current_iintpri]);
		}
	}
}


/*
 *  (モデル上の)割込み優先度マスクの設定
 *
 *  本OSでは次の二点が成り立つ
 *    * OS割込み禁止状態で呼び出される
 *    * OS割込み禁止時の優先度より高い値は指定されない
 *  OS割込み禁止はPMRで実現されており，ここでPMRを変更すると，OS割込み禁止
 *  が解除される場合があるため，内部変数(current_iintpri) の変更のみを行う．
 *  優先度マスクの変更は，APIの出口で呼び出される，x_nested_unlock_os_int()
 *  で実施される．
 */
LOCAL_INLINE void
x_set_ipm(PriorityType intpri)
{
	ASSERT(nested_lock_os_int_cnt > 0U);

	current_iintpri = INT_IPM(intpri);
}

/*
 *  (モデル上の)割込み優先度マスクの参照
 *
 *  本OSでは次の点が成り立つ
 *    * OS割込み禁止状態で呼び出される
 */
LOCAL_INLINE PriorityType
x_get_ipm(void)
{
	ASSERT(nested_lock_os_int_cnt > 0U);
	return(EXT_IPM(current_iintpri));
}

/*
 *  割込み要求禁止フラグのセット
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */
LOCAL_INLINE boolean
x_disable_int(InterruptNumberType intno)
{
	uint32 eic_address = EIC_ADDRESS(intno);

	if (!VALID_INTNO(intno)) {
		return(FALSE);
	}

	/* 割込みの禁止(7bit目をセット) */
	sil_wrh_mem((void *) eic_address,
				sil_reh_mem((void *) eic_address) | (0x01U << 7));

	return(TRUE);
}
/*
 *  割込み要求禁止フラグの解除
 *
 *  割込み属性が設定されていない割込み要求ラインに対して割込み要求禁止
 *  フラグをクリアしようとした場合には，falseを返す．
 */

LOCAL_INLINE boolean
x_enable_int(InterruptNumberType intno)
{
	uint32 eic_address = EIC_ADDRESS(intno);

	if (!VALID_INTNO(intno)) {
		return(FALSE);
	}

	/* 7bit目をクリア */
	sil_wrh_mem((void *) eic_address,
				sil_reh_mem((void *) eic_address) & ~(0x01U << 7));

	return(TRUE);
}

/*
 *  割込み要求のクリア
 */
LOCAL_INLINE boolean
x_clear_int(InterruptNumberType intno)
{
	uint32 eic_address = EIC_ADDRESS(intno);

	if (!VALID_INTNO(intno)) {
		return(FALSE);
	}

	/* 割込みのクリア(12bit目をクリア) */
	sil_wrh_mem((void *) eic_address,
				sil_reh_mem((void *) eic_address) & ~(0x01U << 12));

	return(TRUE);
}

/*
 *  割込み要求のチェック
 */
LOCAL_INLINE boolean
x_probe_int(InterruptNumberType intno)
{
	uint32	eic_address = EIC_ADDRESS(intno);
	uint16	tmp;

	if (!VALID_INTNO(intno)) {
		return(FALSE);
	}

	tmp = sil_reh_mem((void *) (eic_address));
	return((tmp & 0x1000) != 0);
}

/*
 *  割込み要求ラインの属性の設定
 */
extern void x_config_int(InterruptNumberType intno, AttributeType intatr, PriorityType intpri);

/*
 *  割込みハンドラの入り口で必要なIRC操作
 */
LOCAL_INLINE void
i_begin_int(InterruptNumberType intno)
{

}

/*
 *  割込みハンドラの出口で必要なIRC操作
 */
LOCAL_INLINE void
i_end_int(InterruptNumberType intno)
{

}

/*
 *  未定義の割込みが入った場合の処理
 */
extern void default_int_handler(void);
extern void _start(void);
extern void prc_hardware_initialize(void);

/*
 *  プロセッサ依存の初期化
 */
extern void prc_initialize(void);

/*
 *  プロセッサ依存の終了時処理
 */
extern void prc_terminate(void);

/*
 *  タスクディスパッチャ
 */

/*
 *  最高優先順位タスクへのディスパッチ（prc_support.S）
 *
 *  dispatch は，タスクコンテキストから呼び出されたサービスコール処理
 *  内で，OS割込み禁止状態で呼び出さなければならない
 */
extern void dispatch(void);

/*
 *  ディスパッチャの動作開始（prc_support.S）
 *
 *  start_dispatchは，カーネル起動時に呼び出すべきもので，すべての割込
 *  みを禁止した状態（全割込み禁止状態と同等の状態）で呼び出さなければ
 *  ならない
 */
extern void start_dispatch(void) NoReturn;

/*
 *  現在のコンテキストを捨ててディスパッチ（prc_support.S）
 *
 *  exit_and_dispatch は，OS割込み禁止状態で呼び出さなければならない
 */
extern void exit_and_dispatch(void) NoReturn;

/* =begin added for SC3 */
/*
 *  現在のコンテキストを捨ててディスパッチ（prc_support.S）
 *   PosttaskHook を呼ばないエントリ
 *
 *  exit_and_dispatch_nohook は，OS割込み禁止状態で呼び出さなければならない
 */
extern void exit_and_dispatch_nohook(void) NoReturn;
/* =end added for SC3 */

/*
 *  タスクコンテキストブロックの定義
 */
typedef struct task_context_block {
    /* =begin modified for SC3 */
	void	*ssp;               /* システムスタックポインタ */
	void	*usp;               /* ユーザスタックポインタ */
    boolean priv_mode;          /* 特権モード実行中か */
    /* =end modified for SC3 */
	FunctionRefType	pc;         /* プログラムカウンタ */
} TSKCTXB;

/* =begin modified for SC3 */
/*
 *	タスク初期化コンテキストブロック
 */
#define USE_TSKINICTXB	/*  TSKINICTXBを使用する  */

typedef struct task_initialization_context_block {
	MemorySizeType sstksz;  /* スタック領域のサイズ（丸めた値） */
	void	*sstk_bottom;	/* スタックポインタの初期値（スタックの底の初期値） */
	MemorySizeType stksz;   /* スタック領域のサイズ（丸めた値） */
	void	*stk_bottom;	/* スタックポインタの初期値（スタックの底の初期値） */
} TSKINICTXB;

/* 
 *  スタックのMPU情報
 */
typedef struct task_stack_mpu_info_block {
    uint8 *start_ustk_label;
    uint8 *limit_ustk_label;
} STKMPUINFOB;

/*
 *  OSアプリケーションのMPU情報
 */
typedef struct {
    uint32 mpul;
    uint32 mpua;
    uint32 mpat;
} MPUINFOB;

typedef struct {
#ifdef __v850e2v3__
    uint8 *start_text_label;
    uint8 *limit_text_label;
    uint8 *start_rosdata_label;
    uint8 *limit_rosdata_label;
    uint8 *start_ram_label;
    uint8 *limit_ram_label;
    uint8 *start_sram_label;
    uint8 *limit_sram_label;
#else /* __v850e2v3__ */
#ifdef USE_MPU_SRPW_DATA
    uint8 *start_srpw_label;
    uint8 *limit_srpw_label;
#endif /* USE_MPU_SRPW_DATA */
#ifdef USE_MPU_SRPW_SDATA
    uint8 *start_ssrpw_label;
    uint8 *limit_ssrpw_label;
#endif /* USE_MPU_SRPW_SDATA */
#ifdef USE_MPU_PR_TEXT
    uint8 *start_text_label;
    uint8 *limit_text_label;
#endif /* USE_MPU_PR_TEXT */
#ifdef USE_MPU_PR_SDATA
    uint8 *start_rosdata_label;
    uint8 *limit_rosdata_label;
#endif /* USE_MPU_PR_SDATA */
#ifdef USE_MPU_PRW_DATA
    uint8 *start_ram_label;
    uint8 *limit_ram_label;
#endif /* USE_MPU_PRW_DATA */
#ifdef USE_MPU_PRW_SDATA
    uint8 *start_sram_label;
    uint8 *limit_sram_label;
#endif /* USE_MPU_PRW_SDATA */
    MPUINFOB *mpu_area_info;
    uint8 tnum_mpu_area;
    uint32 mprc;
#endif /* __v850e2v3__ */
} OSAPMPUINFOB;
/* =end modified for SC3 */

/*
 *  タスクコンテキストの初期化
 *
 *  タスクが休止状態から実行できる状態に移行する時に呼ばれる．この時点
 *  でスタック領域を使ってはならない
 *
 *  activate_contextを，インライン関数ではなくマクロ定義としているのは，
 *  この時点ではTCBが定義されていないためである
 */
/* =begin modified for SC3 */
extern void    start_stask_r(void);
extern void    start_utask_r(void);

#define activate_context(p_tcb)                                         \
{\
	(p_tcb)->tskctxb.ssp = (p_tcb)->p_tinib->tskinictxb.sstk_bottom;	\
	(p_tcb)->tskctxb.pc = ((p_tcb)->p_tinib->p_osapcb->p_osapinib->osap_trusted != FALSE) ? (FunctionRefType)start_stask_r: (FunctionRefType)start_utask_r;       \
	(p_tcb)->tskctxb.priv_mode = 0U; \
}

/* 
 *  force_term_osap_mainの起動 
 *  リスタートタスクのpcにforce_term_osap_mainのアドレスを格納する 
 */ 
#ifdef CFG_USE_STACKMONITORING
#define activate_force_term_osap_main(p_tcb)                            \
{\
	(p_tcb)->tskctxb.ssp = (p_tcb)->p_tinib->tskinictxb.sstk_bottom;	\
    *(uint32 *)((uint32)((p_tcb)->tskctxb.ssp) - ((p_tcb)->p_tinib->tskinictxb.sstksz)) = STACK_MAGIC_NUMBER;	\
    (p_tcb)->tskctxb.pc = &(force_term_osap_main);                      \
}
#else /*CFG_USE_STACKMONITORING*/
#define activate_force_term_osap_main(p_tcb)                            \
{\
	(p_tcb)->tskctxb.ssp = (p_tcb)->p_tinib->tskinictxb.sstk_bottom;	\
    (p_tcb)->tskctxb.pc = &(force_term_osap_main);                      \
}
#endif /*CFG_USE_STACKMONITORING*/

#define TOPPERS_SSTK_MAGIC_REGION(p_tinib)	((StackType *) ((uint32)(p_tinib)->tskinictxb.sstk_bottom - (uint32)(p_tinib)->tskinictxb.sstksz))
/* =end modified for SC3 */

/* 引数の型はハードウェアが扱えるサイズ（uint32）と同サイズを使用 */
extern void stack_change_and_call_func_1(void (*func)(StatusType ercd), uint32 arg1);
extern void stack_change_and_call_func_2(void (*func)(StatusType ercd, OSServiceIdType svcid), uint8 arg1, uint8 arg2);

/* 
 *  850では全ての割込み要求ラインに対して 
 *  有効/無効を制御可能であるため，常にtrueを返す 
 */ 
#define target_is_int_controllable(intno) TRUE

#ifdef ENABLE_RETURN_MAIN
/*
 *  AKTSP用のmain()へのリターンコード（prc_support.S）
 */
extern void return_main(void);
#endif /* ENABLE_RETURN_MAIN */

/* =begin modified for SC3 */
#define call_protectionhk_main_stkchg(ercd)	stack_change_and_call_func_1(&call_protectionhk_main, ((uint32) (ercd)))

extern StatusType trustedfunc_stack_check(MemorySizeType size);
extern StatusType no_support_service(void);

/*
 * OS割込み禁止時のstatus.ILへの設定値
 */
extern const uint32	tmin_status_il;

/*
 *  x_nested_lock_os_int()のネスト回数
 *  アクセスの順序が変化しないよう，volatile を指定
 */
extern volatile uint8	nested_lock_os_int_cnt;

LOCAL_INLINE void
x_clear_nested_os_int(void)
{
	ASSERT(nested_lock_os_int_cnt > 0U);
	nested_lock_os_int_cnt = 1U;
}

#define check_address_sstack(base, size, p_tinib) (check_address_stack(base, size, (const MemoryStartAddressType)((uint32)(p_tinib)->tskinictxb.sstk_bottom - (uint32)(p_tinib)->tskinictxb.sstksz), (p_tinib)->tskinictxb.sstksz))
#define check_address_ustack(base, size, p_tinib) (check_address_stack(base, size, (const MemoryStartAddressType)((uint32)(p_tinib)->tskinictxb.stk_bottom - (uint32)(p_tinib)->tskinictxb.stksz), (p_tinib)->tskinictxb.stksz))

/* =end modified for SC3 */

#endif /* TOPPERS_MACRO_ONLY */

/* =begin modified for SC3 */
/*
 *  syscallのパラメータ
 */
#define NO_SVC          1   /* サービスコール呼び出し時のsvcの引数 */

#ifdef USE_KERNEL_LIBRARY_SYSLOG
#include "prc_syslog.h"
#endif /* USE_KERNEL_LIBRARY_SYSLOG */
/* =end modified for SC3 */

#endif /* TOPPERS_PRC_CONFIG_H */
