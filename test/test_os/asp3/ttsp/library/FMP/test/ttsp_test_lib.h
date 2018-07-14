/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
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
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: ttsp_test_lib.h 2 2012-05-09 02:23:52Z nces-shigihara $
 */

/* 
 *		テストプログラム用ライブラリ
 */

#ifndef TTSP_TEST_LIB_H
#define TTSP_TEST_LIB_H

#include <t_stddef.h>
#include <queue.h>

#include <kernel/kernel_impl.h>
#include <kernel/check.h>
#include <kernel/task.h>
#include <kernel/wait.h>
#include <kernel/semaphore.h>
#include <kernel/eventflag.h>
#include <kernel/dataqueue.h>
#include <kernel/pridataq.h>
#include <kernel/spin_lock.h>
#include <kernel/mailbox.h>
#include <kernel/mempfix.h>
#include <kernel/time_event.h>
#include <kernel/alarm.h>
#include <kernel/cyclic.h>
#include <kernel/mp.h>
#include <target_ipi.h>

#define TTSP_FMP   /* FMPのみサポートするため */

#define TTS_RUS		UINT_C(0x20)	/* 強制待ち状態［実行継続中］ */

/*
 *	自己診断関数の型
 */
typedef ER (*BIT_FUNC)(void);

/*
 *	自己診断関数の設定
 */
extern void	set_bit_func(BIT_FUNC bit_func);

/*
 *  テストライブラリ用変数初期化
 */
extern void	ttsp_initialize_test_lib(void);

/*
*	チェックポイント(ASPのAPIテスト流用のみ使用)
 */
extern void	ttsp_check_point(uint_t count);

/*
 *	完了チェックポイント(ASPのAPIテスト流用のみ使用)
 */
extern void	ttsp_check_finish(uint_t count);

/*
 *	チェックポイント通過の状態取得
 */
extern bool_t	ttsp_get_cp_state(void);

/*
 *	チェックポイント通過の状態設定
 */
extern void	ttsp_set_cp_state(bool_t state);

/*
 *	条件チェック
 */
extern void	_check_assert(const char *expr, const char *file, int_t line);
#define check_assert(exp) \
	((void)(!(exp) ? (_check_assert(#exp, __FILE__, __LINE__), 0) : 0))

/*
 *	エラーコードチェック
 */
extern void	_check_ercd(ER ercd, const char *file, int_t line);
#define check_ercd(ercd, expected_ercd) \
	((void)((ercd) != (expected_ercd) ? \
					(_check_ercd(ercd, __FILE__, __LINE__), 0) : 0))


/*
 *  コンテキストに応じたCPUロック
 */
Inline bool_t
ttsp_loc_cpu(void)
{
	bool_t	locked = false;

	if (!sense_context()) {
		if (!t_sense_lock()) {
			t_lock_cpu();
			locked = true;
		}
	}
	else {
		if (!i_sense_lock()) {
			i_lock_cpu();
			locked = true;
		}
	}

	return locked;
}

/*
 *  コンテキストに応じたCPUロック解除
 */
Inline void
ttsp_unl_cpu(bool_t locked)
{
	if (locked) {
		if (!sense_context()) {
			t_unlock_cpu();
		}
		else {
			i_unlock_cpu();
		}
	}
}


/*
 *  バリア同期
 */
extern void ttsp_barrier_sync(uint_t phase, uint_t tnum_prcid);

/*
 *  ID指定のチェックポイント
 *  IDの最大値はプロセッサIDと同じ
 */

/*  
 *  id番号のチェックポイントを進める． 
 */
extern void ttsp_mp_check_point(ID id, uint_t count);

/*
 *  id番号のチェックポイントがcountになるのを待つ． 
 */
extern void ttsp_mp_wait_check_point(ID id, uint_t count);

/*
 *  id番号のチェックポイントを進めて，カーネルを終了させる．
 */ 
extern void ttsp_mp_check_finish(ID id, uint_t count);

/*
 *  対象タスクの状態が指定した状態へ変化するのを待つ．
 */ 
extern void ttsp_state_sync(char* proc_id, char* target_id, ID target_tskid, char* target_state_id, STAT target_state);

/*
 *  実行状態のまま他タスクからter_tskにより終了されるのを待つ．
 */ 
extern void ttsp_wait_finish_sync(char* proc_id);



/*
 *  ロック取得の施行時に割込みを許可しないロック取得関数
 *  (代替関数はCPUロック状態でも発行可能とするため)
 */
#if TTYPE_KLOCK == G_KLOCK

Inline PCB*
ttsp_acquire_tsk_lock_without_preemption(TCB *p_tcb)
{
	x_acquire_lock_without_preemption(&giant_lock);
	return(p_tcb->p_pcb);
}

Inline PCB*
ttsp_acquire_tsk_lock_without_preemption_cyc(CYCCB *p_cyccb)
{
	x_acquire_lock_without_preemption(&giant_lock);
	return(p_cyccb->p_pcb);
}

Inline PCB*
ttsp_acquire_tsk_lock_without_preemption_alm(ALMCB *p_almcb)
{
	x_acquire_lock_without_preemption(&giant_lock);
	return(p_almcb->p_pcb);
}

Inline void
ttsp_acquire_obj_lock_without_preemption(LOCK *p_objlock)
{
	x_acquire_lock_without_preemption(&giant_lock);
}

#else /* P_KLOCK or F_KLOCK */

Inline PCB*
ttsp_acquire_tsk_lock_without_preemption(TCB *p_tcb)
{
	PCB *p_pcb;

	while(true) {
		p_pcb = p_tcb->p_pcb;
		x_acquire_lock_without_preemption(&(p_pcb->tsk_lock));
		if (p_pcb != p_tcb->p_pcb) {
			/* 対象タスクがマイグレートした場合 */
			x_release_lock(&(p_pcb->tsk_lock));
		} else {
			break;
		}
	}

	return(p_tcb->p_pcb);
}

Inline PCB*
ttsp_acquire_tsk_lock_without_preemption_cyc(CYCCB *p_cyccb)
{
	PCB *p_pcb;

	while(true) {
		p_pcb = p_cyccb->p_pcb;
		x_acquire_lock_without_preemption(&(p_pcb->tsk_lock));
		if (p_pcb != p_cyccb->p_pcb) {
			/* 対象タスクがマイグレートした場合 */
			x_release_lock(&(p_pcb->tsk_lock));
		} else {
			break;
		}
	}

	return(p_cyccb->p_pcb);
}

Inline PCB*
ttsp_acquire_tsk_lock_without_preemption_alm(ALMCB *p_almcb)
{
	PCB *p_pcb;

	while(true) {
		p_pcb = p_almcb->p_pcb;
		x_acquire_lock_without_preemption(&(p_pcb->tsk_lock));
		if (p_pcb != p_almcb->p_pcb) {
			/* 対象タスクがマイグレートした場合 */
			x_release_lock(&(p_pcb->tsk_lock));
		} else {
			break;
		}
	}

	return(p_almcb->p_pcb);
}

Inline void
ttsp_acquire_obj_lock_without_preemption(LOCK *p_objlock)
{
	x_acquire_lock_without_preemption(p_objlock);
}

#endif /* TTYPE_KLOCK == G_KLOCK */


/*
 *  ref_tsk代替関数
 */
typedef struct t_ttsp_rtsk {
	STAT		tskstat;		/* タスク状態 */
	PRI			tskpri;			/* タスクの現在優先度 */
	PRI			tskbpri;		/* タスクのベース優先度 */
	STAT		tskwait;		/* 待ち要因 */
	ID			wobjid;			/* 待ち対象のオブジェクトのID */
	TMO			lefttmo;		/* タイムアウトするまでの時間 */
	uint_t		actcnt;			/* 起動要求キューイング数 */
	uint_t		wupcnt;			/* 起床要求キューイング数 */
	ATR			tskatr;			/* タスク属性 */
	intptr_t	exinf;			/* タスクの拡張情報 */
	PRI			itskpri;		/* タスクの起動時優先度 */
	SIZE		stksz;			/* スタック領域のサイズ */
	void		*stk;			/* スタック領域の先頭番地 */
	uint_t		porder;			/* 割付けプロセッサの同一優先度タスク内での優先順位 */
	ID			prcid;			/* 割付けプロセッサのID */
	ID			actprc;			/* 次の起動時の割付けプロセッサのID */
	ID			iaffinity;		/* タスクの初期割付けプロセッサ */
	uint_t		affinity_mask;	/* タスクの割付け可能プロセッサ */
} T_TTSP_RTSK;

extern ER ttsp_ref_tsk(ID tskid, T_TTSP_RTSK *pk_rtsk);



/*
 *  ref_tex代替関数
 */
typedef struct t_ttsp_rtex {
	STAT	texstat;	/* タスク例外処理の状態 */
	TEXPTN	pndptn;		/* 保留例外要因 */
	ATR		texatr;		/* タスク例外処理ルーチン属性 */
} T_TTSP_RTEX;

extern ER ttsp_ref_tex(ID tskid, T_TTSP_RTEX *pk_rtex);



/*
 *  ref_sem代替関数
 */
typedef struct t_ttsp_rsem {
	uint_t	semcnt;		/* セマフォの資源数 */
	ATR		sematr;		/* セマフォ属性 */
	uint_t	isemcnt;	/* セマフォの初期資源数 */
	uint_t	maxsem;		/* セマフォの最大資源数 */
	uint_t	waitcnt;	/* 待ちタスクの数 */
} T_TTSP_RSEM;

extern ER ttsp_ref_sem(ID semid, T_TTSP_RSEM *pk_rsem);


/*
 *  セマフォの待ちタスク参照関数
 */
extern ER ttsp_ref_wait_sem(ID semid, uint_t order, ID *p_tskid);



/*
 *  ref_flg代替関数
 */
typedef struct t_ttsp_rflg {
	FLGPTN	flgptn;		/* イベントフラグの現在のビットパターン */
	ATR		flgatr;		/* イベントフラグ属性 */
	FLGPTN	iflgptn;	/* イベントフラグのビットパターンの初期値 */
	uint_t	waitcnt;	/* 待ちタスクの数 */
} T_TTSP_RFLG;

extern ER ttsp_ref_flg(ID flgid, T_TTSP_RFLG *pk_rflg);


/*
 *  イベントフラグの待ちタスク参照関数
 */
extern ER ttsp_ref_wait_flg(ID flgid, uint_t order, ID *p_tskid, FLGPTN *p_waiptn, MODE *p_wfmode);


/*
 *  ref_dtq代替関数
 */
typedef struct t_ttsp_rdtq {
	uint_t	sdtqcnt;	/* データキュー管理領域に格納されているデータの数 */
	ATR		dtqatr;		/* データキュー属性 */
	uint_t	dtqcnt;		/* データキューの容量 */
	uint_t	swaitcnt;	/* 送信待ちタスクの数 */
	uint_t	rwaitcnt;	/* 受信待ちタスクの数 */
} T_TTSP_RDTQ;

extern ER ttsp_ref_dtq(ID dtqid, T_TTSP_RDTQ *pk_rdtq);


/*
 *  データキュー管理領域に格納されているデータ参照関数
 */
extern ER ttsp_ref_data(ID dtqid, uint_t index, intptr_t *p_data);


/*
 *  データキューの送信待ちタスク参照関数
 */
extern ER ttsp_ref_swait_dtq(ID dtqid, uint_t order, ID *p_tskid, intptr_t *p_data);


/*
 *  データキューの受信待ちタスク参照関数
 */
extern ER ttsp_ref_rwait_dtq(ID dtqid, uint_t order, ID *p_tskid);



/*
 *  ref_pdq代替関数
 */
typedef struct t_ttsp_rpdq {
	uint_t	spdqcnt;		/* 優先度データキュー管理領域に格納されているデータの数 */
	ATR		pdqatr;			/* 優先度データキュー属性 */
	uint_t	pdqcnt;			/* 優先度データキューの容量 */
	PRI		maxdpri;		/* データ優先度の最大値 */
	uint_t	swaitcnt;		/* 送信待ちタスクの数 */
	uint_t	rwaitcnt;		/* 受信待ちタスクの数 */
} T_TTSP_RPDQ;

extern ER ttsp_ref_pdq(ID pdqid, T_TTSP_RPDQ *pk_rpdq);


/*
 *  優先度データキュー管理領域に格納されているデータ参照関数
 */
extern ER ttsp_ref_pridata(ID pdqid, uint_t index, intptr_t *p_data, PRI *p_datapri);


/*
 *  優先度データキューの送信待ちタスク参照関数
 */
extern ER ttsp_ref_swait_pdq(ID pdqid, uint_t order, ID *p_tskid, intptr_t *p_data, PRI *p_datapri);


/*
 *  優先度データキューの受信待ちタスク参照関数
 */
extern ER ttsp_ref_rwait_pdq(ID pdqid, uint_t order, ID *p_tskid);



/*
 *  ref_spn代替関数
 */
typedef struct t_ttsp_rspn {
	STAT	spnstat;	/* スピンロックのロック状態 */
	ATR		spnatr;		/* スピンロック属性 */
} T_TTSP_RSPN;

extern ER ttsp_ref_spn(ID spnid, T_TTSP_RSPN *pk_rspn);



/*
 *  ref_mbx代替関数
 */
typedef struct t_ttsp_rmbx {
	uint_t	msgcnt;		/* メールボックスにつながれているメッセージの数 */
	ATR		mbxatr;		/* メールボックス属性 */
	PRI		maxmpri;	/* メッセージ優先度の最大値 */
	uint_t	rwaitcnt;	/* 受信待ちタスクの数 */
} T_TTSP_RMBX;

extern ER ttsp_ref_mbx(ID mbxid, T_TTSP_RMBX *pk_rmbx);


/*
 *  メールボックスにつながれているデータ参照関数
 */
extern ER ttsp_ref_msg(ID mbxid, uint_t index, T_MSG **pp_msg);


/*
 *  メールボックスの受信待ちタスク参照関数
 */
extern ER ttsp_ref_rwait_mbx(ID mbxid, uint_t order, ID *p_tskid);



/*
 *  ref_mpf代替関数
 */
typedef struct t_ttsp_rmpf {
	uint_t	fblkcnt;	/* 固定長メモリプール領域の空きメモリ領域に割り付けることができる固定長メモリブロックの数 */
	ATR		mpfatr;		/* 固定長メモリプール属性 */
	uint_t	blkcnt;		/* メモリブロック数 */
	uint_t	blksz;		/* メモリブロックのサイズ */
	uint_t	waitcnt;	/* 待ちタスクの数 */
	void	*mpf;		/* 固定長メモリプール領域の先頭番地 */
} T_TTSP_RMPF;

extern ER ttsp_ref_mpf(ID mpfid, T_TTSP_RMPF *pk_rmpf);


/*
 *  固定長メモリプールの待ちタスク参照関数
 */
extern ER ttsp_ref_wait_mpf(ID mpfid, uint_t order, ID *p_tskid);



/*
 *  ref_cyc代替関数
 */
typedef struct t_ttsp_rcyc {
	STAT		cycstat;		/* 周期ハンドラの動作状態 */
	RELTIM		lefttim;		/* 次に周期ハンドラを起動する時刻までの相対時間 */
	ATR			cycatr;			/* 周期ハンドラ属性 */
	intptr_t	exinf;			/* 周期ハンドラの拡張情報 */
	RELTIM		cyctim;			/* 周期ハンドラの起動周期 */
	RELTIM		cycphs;			/* 周期ハンドラの起動位相 */
	ID			prcid;			/* 割付けプロセッサのID */
#ifdef TOPPERS_SYSTIM_LOCAL
	ID			iaffinity;		/* 周期ハンドラの初期割付けプロセッサ */
	uint_t		affinity_mask;	/* 周期ハンドラの割付け可能プロセッサ */
#endif /* TOPPERS_SYSTIM_LOCAL */
} T_TTSP_RCYC;

extern ER ttsp_ref_cyc(ID cycid, T_TTSP_RCYC *pk_rcyc);



/*
 *  ref_alm代替関数
 */
typedef struct t_ttsp_ralm {
	STAT		almstat;		/* アラームハンドラの動作状態 */
	RELTIM		lefttim;		/* アラームハンドラを起動する時刻までの相対時間 */
	ATR			almatr;			/* アラームハンドラ属性 */
	intptr_t	exinf;			/* アラームハンドラの拡張情報 */
	ID			prcid;			/* 割付けプロセッサのID */
#ifdef TOPPERS_SYSTIM_LOCAL
	ID			iaffinity;		/* アラームハンドラの初期割付けプロセッサ */
	uint_t		affinity_mask;	/* アラームハンドラの割付け可能プロセッサ */
#endif /* TOPPERS_SYSTIM_LOCAL */
} T_TTSP_RALM;

extern ER ttsp_ref_alm(ID almid, T_TTSP_RALM *pk_ralm);

/*
 *  get_ipm代替関数
 */
extern ER ttsp_get_ipm(PRI *p_intpri);

/*
 *  sus_tsk代替関数
 */
extern ER ttsp_sus_tsk(ID tskid);

#endif /* TTSP_TEST_LIB_H */
