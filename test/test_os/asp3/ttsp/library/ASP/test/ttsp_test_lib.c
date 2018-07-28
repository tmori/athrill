/*
 *  TTSP
 *      TOPPERS Test Suite Package
 * 
 *  Copyright (C) 2010-2011 by Center for Embedded Computing Systems
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2010-2011 by Digital Craft Inc.
 *  Copyright (C) 2010-2011 by NEC Communication Systems, Ltd.
 *  Copyright (C) 2010-2011 by FUJISOFT INCORPORATED
 *  Copyright (C) 2010-2011 by Industrial Technology Institute,
 *								Miyagi Prefectural Government, JAPAN
 * 
 *  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: ttsp_test_lib.c 2 2012-05-09 02:23:52Z nces-shigihara $
 */

/* 
 *		テストプログラム用ライブラリ
 */

#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "ttsp_test_lib.h"

/*
 *  チェックポイント通過カウント変数
 */
static volatile uint_t	check_count;

/*
 *  自己診断関数
 */
static volatile BIT_FUNC	check_bit_func;

/*
 *  チェックポイント通過の状態(true:正常，false:異常)
 */ 
static volatile bool_t	cp_state;

/*
 *  自己診断関数の設定
 */
void
set_bit_func(BIT_FUNC bit_func)
{
	check_bit_func = bit_func;
}

/*
 *  テストライブラリ用変数初期化
 */
void
ttsp_initialize_test_lib(void)
{
	check_count = 0U;
	check_bit_func = NULL;
	cp_state = true;
}

/*
 *  チェックポイント
 */
void
ttsp_check_point(uint_t count)
{
	bool_t	errorflag = false;
	ER		rercd;
	SIL_PRE_LOC;

	/*
	 *  割込みロック状態に
	 */
	SIL_LOC_INT();

	/*
	 *  シーケンスチェック
	 */
	if (++check_count == count) {
		syslog_1(LOG_NOTICE, "Check point %d passed.", count);
	}
	else {
		syslog_1(LOG_ERROR, "## Unexpected check point %d.", count);
		errorflag = true;
	}

	/*
	 *  カーネルの内部状態の検査
	 */
	if (check_bit_func != NULL) {
		rercd = (*check_bit_func)();
		if (rercd < 0) {
			syslog_2(LOG_ERROR, "## Internal inconsistency detected (%s, %d).",
								itron_strerror(rercd), SERCD(rercd));
			errorflag = true;
		}
	}

	/*
	 *  割込みロック状態を解除
	 */
	SIL_UNL_INT();

	if (errorflag) {
		cp_state = false;
		if (sns_ker() == false) {
			ext_ker();
		}
	}
}

/*
 *  チェックポイントがcountになるのを待つ
 */
void
ttsp_wait_check_point(uint_t count)
{
	ulong_t	timeout = 0;

	while (check_count < count) {
		/*
		 * タイムアウト処理
		 */
		timeout++;
		if (timeout > TTSP_LOOP_COUNT) {
			syslog_1(LOG_ERROR, "## ttsp_wait_check_point(%d) caused a timeout.", count);
			cp_state = false;
			ext_ker();
		}
		sil_dly_nse(TTSP_SIL_DLY_NSE_TIME);
	}
}

/*
 *  完了チェックポイント
 */
void
ttsp_check_finish(uint_t count)
{
	ttsp_check_point(count);
	syslog_0(LOG_NOTICE, "All check points passed.");
#if 1
	// log task waiting..
	while (1) {
		;
	}
#else
	ext_ker();
#endif
}

/*
 *	チェックポイント通過の状態取得
 */
bool_t
ttsp_get_cp_state(void)
{
	return cp_state;
}

/*
 *	チェックポイント通過の状態設定
 */
void
ttsp_set_cp_state(bool_t state)
{
	cp_state = state;
}

/*
 *  条件チェックのエラー処理
 */
void
_check_assert(const char *expr, const char *file, int_t line)
{
	syslog_3(LOG_ERROR, "## Assertion `%s' failed at %s:%u.",
								expr, file, line);
	cp_state = false;
	if (sns_ker() == false) {
		ext_ker();
	}
}

/*
 *  エラーコードチェックのエラー処理
 */
void
_check_ercd(ER ercd, const char *file, int_t line)
{
	syslog_3(LOG_ERROR, "## Unexpected error %s detected at %s:%u.",
								itron_strerror(ercd), file, line);
	cp_state = false;
	if (sns_ker() == false) {
		ext_ker();
	}
}


/*
 *  ref_tsk代替関数
 */
ER
ttsp_ref_tsk(ID tskid, T_TTSP_RTSK *pk_rtsk)
{
	TCB		*p_tcb;
	uint_t	tstat;
	ER		ercd;
	uint_t	porder;
	QUEUE	*p_next;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_TSKID(tskid));
	p_tcb = &_kernel_tcb_table[tskid - 1];

	tstat = p_tcb->tstat;
	if (TSTAT_DORMANT(tstat)) {
		/*
		 *  対象タスクが休止状態の場合
		 */
		pk_rtsk->tskstat = TTS_DMT;
	}
	else {
		/*
		 *  タスク状態の取出し
		 */
		if (TSTAT_SUSPENDED(tstat)) {
			if (TSTAT_WAITING(tstat)) {
				pk_rtsk->tskstat = TTS_WAS;
			}
			else {
				pk_rtsk->tskstat = TTS_SUS;
			}
		}
		else if (TSTAT_WAITING(tstat)) {
			pk_rtsk->tskstat = TTS_WAI;
		}
		else if (p_tcb == p_runtsk) {
			pk_rtsk->tskstat = TTS_RUN;
		}
		else {
			pk_rtsk->tskstat = TTS_RDY;
		}

		/*
		 *  現在優先度とベース優先度の取出し
		 */
		pk_rtsk->tskpri = EXT_TSKPRI(p_tcb->priority);
		pk_rtsk->tskbpri = EXT_TSKPRI(p_tcb->priority);

		if (TSTAT_WAITING(tstat)) {
			/*
			 *  待ち要因と待ち対象のオブジェクトのIDの取出し
			 */
			switch (tstat & TS_WAITING_MASK) {
			case TS_WAITING_SLP:
				pk_rtsk->tskwait = TTW_SLP;
				break;
			case TS_WAITING_DLY:
				pk_rtsk->tskwait = TTW_DLY;
				break;
			case TS_WAITING_SEM:
				pk_rtsk->tskwait = TTW_SEM;
				pk_rtsk->wobjid = SEMID(((WINFO_SEM *)(p_tcb->p_winfo))
																->p_semcb);
				break;
			case TS_WAITING_FLG:
				pk_rtsk->tskwait = TTW_FLG;
				pk_rtsk->wobjid = FLGID(((WINFO_FLG *)(p_tcb->p_winfo))
																->p_flgcb);
				break;
			case TS_WAITING_SDTQ:
				pk_rtsk->tskwait = TTW_SDTQ;
				pk_rtsk->wobjid = DTQID(((WINFO_SDTQ *)(p_tcb->p_winfo))
																->p_dtqcb);
				break;
			case TS_WAITING_RDTQ:
				pk_rtsk->tskwait = TTW_RDTQ;
				pk_rtsk->wobjid = DTQID(((WINFO_RDTQ *)(p_tcb->p_winfo))
																->p_dtqcb);
				break;
			case TS_WAITING_SPDQ:
				pk_rtsk->tskwait = TTW_SPDQ;
				pk_rtsk->wobjid = PDQID(((WINFO_SPDQ *)(p_tcb->p_winfo))
																->p_pdqcb);
				break;
			case TS_WAITING_RPDQ:
				pk_rtsk->tskwait = TTW_RPDQ;
				pk_rtsk->wobjid = PDQID(((WINFO_RPDQ *)(p_tcb->p_winfo))
																->p_pdqcb);
				break;
#if 0				
			case TS_WAIT_MBX:
				pk_rtsk->tskwait = TTW_MBX;
				pk_rtsk->wobjid = MBXID(((WINFO_MBX *)(p_tcb->p_winfo))
																->p_mbxcb);
				break;
#endif
			case TS_WAITING_MPF:
				pk_rtsk->tskwait = TTW_MPF;
				pk_rtsk->wobjid = MPFID(((WINFO_MPF *)(p_tcb->p_winfo))
																->p_mpfcb);
				break;
			}

			/*
			 *  タイムアウトするまでの時間の取出し
			 */
			if (p_tcb->p_winfo->p_tmevtb != NULL) {
				pk_rtsk->lefttmo
						= (TMO) tmevt_lefttim(p_tcb->p_winfo->p_tmevtb);
			}
			else {
				pk_rtsk->lefttmo = TMO_FEVR;
			}
		}

		/*
 		 *  起床要求キューイング数の取出し
		 */
		pk_rtsk->wupcnt = p_tcb->wupque ? 1U : 0U;
	}

	/*
	 *  起動要求キューイング数の取出し
	 */
	pk_rtsk->actcnt = p_tcb->actque ? 1U : 0U;

	/*
	 *  タスク属性の取出し
	 */
	pk_rtsk->tskatr = p_tcb->p_tinib->tskatr;

	/*
	 *  タスクの拡張情報の取出し
	 */
	pk_rtsk->exinf = p_tcb->p_tinib->exinf;

	/*
	 *  タスクの起動時優先度の取出し
	 */
	pk_rtsk->itskpri = EXT_TSKPRI(p_tcb->p_tinib->ipriority);

	/*
	 *  スタック領域のサイズの取出し
	 */
#ifdef USE_TSKINICTXB
	/*
	 *  USE_TSKINICTXBを定義しているターゲットでは
	 *  標準形式への変換マクロ／変数を用意する
	 */
	pk_rtsk->stksz = ttsp_target_get_stksz(p_tcb->p_tinib);
#else /* USE_TSKINICTXB */
	pk_rtsk->stksz = p_tcb->p_tinib->stksz;
#endif /* USE_TSKINICTXB */

	/*
	 *  スタック領域の先頭番地の取出し
	 */
#ifdef USE_TSKINICTXB
	pk_rtsk->stk = ttsp_target_get_stk(p_tcb->p_tinib);
#else /* USE_TSKINICTXB */
	pk_rtsk->stk = p_tcb->p_tinib->stk;
#endif /* USE_TSKINICTXB */

	/*
	 *  同一優先度タスク内での優先順位算出
	 *  ("MAIN_TASK(=1)"はカウント対象外とする)
	 */
	if ((TTS_RUN == pk_rtsk->tskstat) || (TTS_RDY == pk_rtsk->tskstat)) {
		porder = 0;
		p_next = NULL;
		p_next = ready_queue[pk_rtsk->tskpri - 1].p_next;
		do {
			if (TSKID((TCB *)p_next) != 1) {
				porder++;
			}
			if (TSKID((TCB *)p_next) == tskid) {
				break;
			}
			p_next = p_next->p_next;
		} while (&ready_queue[pk_rtsk->tskpri - 1] != p_next);

		pk_rtsk->porder = porder;
	}

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}

#if 0
/*
 *  ref_tex代替関数
 */
ER 
ttsp_ref_tex(ID tskid, T_TTSP_RTEX *pk_rtex)
{
	TCB		*p_tcb;
	ER		ercd;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_TSKID(tskid);
	p_tcb = &_kernel_tcb_table[tskid - 1];

	if (TSTAT_DORMANT(p_tcb->tstat) || p_tcb->p_tinib->texrtn == NULL) {
		/*
		 *  タスクが休止状態，タスクに対してタスク例外処理
		 *  ルーチンが定義されていないの場合
		 */
		ercd = E_OBJ;
	}
	else {
		/*
		 *  タスク例外処理の状態，保留例外要因，
		 *  タスク例外処理ルーチン属性を取得
		 */
		pk_rtex->texstat = (p_tcb->enatex) ? TTEX_ENA : TTEX_DIS;
		pk_rtex->pndptn = p_tcb->texptn;
		pk_rtex->texatr = p_tcb->p_tinib->texatr;
		ercd = E_OK;
	}

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}
#endif

/*
 *  ref_sem代替関数
 */
ER
ttsp_ref_sem(ID semid, T_TTSP_RSEM *pk_rsem)
{
	SEMCB	*p_semcb;
	ER		ercd;
	bool_t	locked;
	uint_t	waitcnt;
	QUEUE	*p_next;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_SEMID(semid));
	p_semcb = &_kernel_semcb_table[semid - 1];

	/*
	 *  セマフォの資源数，セマフォ属性，
	 *  セマフォの初期資源数，セマフォの最大資源数を取得
	 */
	pk_rsem->semcnt = p_semcb->semcnt;
	pk_rsem->sematr = p_semcb->p_seminib->sematr;
	pk_rsem->isemcnt = p_semcb->p_seminib->isemcnt;
	pk_rsem->maxsem = p_semcb->p_seminib->maxsem;
	
	/*
	 *  待ちタスクの数算出
	 */
	waitcnt = 0;
	if (wait_tskid(&(p_semcb->wait_queue)) != TSK_NONE) {
		p_next = p_semcb->wait_queue.p_next;
		while (&(p_semcb->wait_queue) != p_next) {
			waitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rsem->waitcnt = waitcnt;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  セマフォの待ちタスク参照関数
 */
ER
ttsp_ref_wait_sem(ID semid, uint_t order, ID *p_tskid)
{
	SEMCB		*p_semcb;
	ER			ercd;
	bool_t		locked;
	T_TTSP_RSEM	ref_rsem;
	QUEUE		*p_next;
	uint_t		i;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_SEMID(semid));
	p_semcb = &_kernel_semcb_table[semid - 1];

	/*
	 *  待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_sem(semid, &ref_rsem);
	check_ercd(ercd, E_OK);
	if ((ref_rsem.waitcnt == 0) || (ref_rsem.waitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで待ちとなっているタスクIDを取得
	 */
	i = 1;
	p_next = &(p_semcb->wait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);
	
	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  ref_flg代替関数
 */
ER
ttsp_ref_flg(ID flgid, T_TTSP_RFLG *pk_rflg)
{
	FLGCB	*p_flgcb;
	ER		ercd;
	bool_t	locked;
	uint_t	waitcnt;
	QUEUE	*p_next;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_FLGID(flgid));
	p_flgcb = &_kernel_flgcb_table[flgid - 1];

	/*
	 *  イベントフラグの現在のビットパターン，イベントフラグ属性，
	 *  イベントフラグのビットパターンの初期値を取得
	 */
	pk_rflg->flgptn = p_flgcb->flgptn;
	pk_rflg->flgatr = p_flgcb->p_flginib->flgatr;
	pk_rflg->iflgptn = p_flgcb->p_flginib->iflgptn;

	/*
	 *  待ちタスクの数
	 */
	waitcnt = 0;
	if (wait_tskid(&(p_flgcb->wait_queue)) != TSK_NONE) {
		p_next = p_flgcb->wait_queue.p_next;
		while (&(p_flgcb->wait_queue) != p_next) {
			waitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rflg->waitcnt = waitcnt;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  イベントフラグの待ちタスク参照関数
 */
ER
ttsp_ref_wait_flg(ID flgid, uint_t order, ID *p_tskid, FLGPTN *p_waiptn, MODE *p_wfmode)
{
	FLGCB		*p_flgcb;
	ER			ercd;
	bool_t		locked;
	T_TTSP_RFLG	ref_rflg;
	QUEUE		*p_next;
	uint_t		i;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_FLGID(flgid));
	p_flgcb = &_kernel_flgcb_table[flgid - 1];

	/*
	 *  待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_flg(flgid, &ref_rflg);
	check_ercd(ercd, E_OK);
	if ((ref_rflg.waitcnt == 0) || (ref_rflg.waitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで待ちとなっているタスクID，待ちビットパターン，データ情報を取得
	 */
	i = 1;
	p_next = &(p_flgcb->wait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);
	*p_waiptn = ((WINFO_FLG *)(((TCB *)(p_next->p_next))->p_winfo))->waiptn;
	*p_wfmode = ((WINFO_FLG *)(((TCB *)(p_next->p_next))->p_winfo))->wfmode;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  ref_dtq代替関数
 */
ER
ttsp_ref_dtq(ID dtqid, T_TTSP_RDTQ *pk_rdtq)
{
	DTQCB	*p_dtqcb;
	ER		ercd;
	uint_t	swaitcnt;
	uint_t	rwaitcnt;
	QUEUE	*p_next;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = &_kernel_dtqcb_table[dtqid - 1];

	/*
	 *  データキュー管理領域に格納されているデータの数，
	 *  データキュー属性，データキューの容量を取得
	 */
	pk_rdtq->sdtqcnt = p_dtqcb->count;
	pk_rdtq->dtqatr = p_dtqcb->p_dtqinib->dtqatr;
	pk_rdtq->dtqcnt = p_dtqcb->p_dtqinib->dtqcnt;

	/*
	 *  送信待ちタスクの数算出
	 */
	swaitcnt = 0;
	if (wait_tskid(&(p_dtqcb->swait_queue)) != TSK_NONE) {
		p_next = p_dtqcb->swait_queue.p_next;
		while (&(p_dtqcb->swait_queue) != p_next) {
			swaitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rdtq->swaitcnt = swaitcnt;

	/*
	 *  受信待ちタスクの数算出
	 */
	rwaitcnt = 0;
	if (wait_tskid(&(p_dtqcb->rwait_queue)) != TSK_NONE) {
		p_next = p_dtqcb->rwait_queue.p_next;
		while (&(p_dtqcb->rwait_queue) != p_next) {
			rwaitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rdtq->rwaitcnt = rwaitcnt;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  データキュー管理領域に格納されているデータ参照関数
 */
ER
ttsp_ref_data(ID dtqid, uint_t index, intptr_t *p_data)
{
	DTQCB	*p_dtqcb;
	ER		ercd;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = &_kernel_dtqcb_table[dtqid - 1];

	/*
	 *  データキュー管理領域につながれているデータの数をチェック
	 */
	if ((p_dtqcb->count == 0) || (p_dtqcb->count < index)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  indexに格納されているデータ情報を取得
	 *  (データキュー内の順序が入れ替わっていることを考慮する)
	 */
	*p_data = (p_dtqcb->p_dtqinib->p_dtqmb + ((p_dtqcb->head +index - 1) % p_dtqcb->p_dtqinib->dtqcnt))->data;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  データキューの送信待ちタスク参照関数
 */
ER
ttsp_ref_swait_dtq(ID dtqid, uint_t order, ID *p_tskid, intptr_t *p_data)
{
	DTQCB		*p_dtqcb;
	QUEUE		*p_next;
	ER			ercd;
	uint_t		i;
	bool_t		locked;
	T_TTSP_RDTQ	ref_rdtq;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = &_kernel_dtqcb_table[dtqid - 1];

	/*
	 *  送信待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_dtq(dtqid, &ref_rdtq);
	check_ercd(ercd, E_OK);
	if ((ref_rdtq.swaitcnt == 0) || (ref_rdtq.swaitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで送信待ちとなっているタスクID，データ情報を取得
	 */
	i = 1;
	p_next = &(p_dtqcb->swait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);
	*p_data = ((WINFO_SDTQ*)(((TCB *)(p_next->p_next))->p_winfo))->data;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  データキューの受信待ちタスク参照関数
 */
ER
ttsp_ref_rwait_dtq(ID dtqid, uint_t order, ID *p_tskid)
{
	DTQCB		*p_dtqcb;
	QUEUE		*p_next;
	ER			ercd;
	uint_t		i;
	bool_t		locked;
	T_TTSP_RDTQ	ref_rdtq;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = &_kernel_dtqcb_table[dtqid - 1];

	/*
	 *  受信待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_dtq(dtqid, &ref_rdtq);
	check_ercd(ercd, E_OK);
	if ((ref_rdtq.rwaitcnt == 0) || (ref_rdtq.rwaitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで受信待ちとなっているタスクIDを取得
	 */
	i = 1;
	p_next = &(p_dtqcb->rwait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  ref_pdq代替関数
 */
ER
ttsp_ref_pdq(ID pdqid, T_TTSP_RPDQ *pk_rpdq)
{
	PDQCB	*p_pdqcb;
	ER		ercd;
	uint_t	swaitcnt;
	uint_t	rwaitcnt;
	QUEUE	*p_next;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_PDQID(pdqid));
	p_pdqcb = &_kernel_pdqcb_table[pdqid - 1];

	/*
	 *  優先度データキュー管理領域に格納されているデータの数，
	 *  優先度データキュー属性，優先度データキューの容量，
	 *  データ優先度の最大値を取得
	 */
	pk_rpdq->spdqcnt = p_pdqcb->count;
	pk_rpdq->pdqatr = p_pdqcb->p_pdqinib->pdqatr;
	pk_rpdq->pdqcnt = p_pdqcb->p_pdqinib->pdqcnt;
	pk_rpdq->maxdpri = p_pdqcb->p_pdqinib->maxdpri;

	/*
	 *  送信待ちタスクの数算出
	 */
	swaitcnt = 0;
	if (wait_tskid(&(p_pdqcb->swait_queue)) != TSK_NONE) {
		p_next = p_pdqcb->swait_queue.p_next;
		while (&(p_pdqcb->swait_queue) != p_next) {
			swaitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rpdq->swaitcnt = swaitcnt;

	/*
	 *  受信待ちタスクの数算出
	 */
	rwaitcnt = 0;
	if (wait_tskid(&(p_pdqcb->rwait_queue)) != TSK_NONE) {
		p_next = p_pdqcb->rwait_queue.p_next;
		while (&(p_pdqcb->rwait_queue) != p_next) {
			rwaitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rpdq->rwaitcnt = rwaitcnt;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  優先度データキュー管理領域に格納されているデータ参照関数
 */
ER
ttsp_ref_pridata(ID pdqid, uint_t index, intptr_t *p_data, PRI *p_datapri)
{
	PDQCB	*p_pdqcb;
	PDQMB	*p_pdqmb;
	ER		ercd;
	uint_t	i;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_PDQID(pdqid));
	p_pdqcb = &_kernel_pdqcb_table[pdqid - 1];

	/*
	 *  格納されているデータの数をチェック 
	 */
	if ((p_pdqcb->count == 0) || (p_pdqcb->count < index)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  indexに格納されているデータ情報を取得
	 */
	i = 1;
	p_pdqmb = p_pdqcb->p_head;
	while (i < index) {
		p_pdqmb = p_pdqmb->p_next;
		i++;
	}
	*p_data = p_pdqmb->data;
	*p_datapri = p_pdqmb->datapri;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  優先度データキューの送信待ちタスク参照関数
 */
ER
ttsp_ref_swait_pdq(ID pdqid, uint_t order, ID *p_tskid, intptr_t *p_data, PRI *p_datapri)
{
	PDQCB		*p_pdqcb;
	QUEUE		*p_next;
	ER			ercd;
	uint_t		i;
	bool_t		locked;
	T_TTSP_RPDQ	ref_rpdq;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_PDQID(pdqid));
	p_pdqcb = &_kernel_pdqcb_table[pdqid - 1];

	/*
	 *  送信待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_pdq(pdqid, &ref_rpdq);
	check_ercd(ercd, E_OK);
	if ((ref_rpdq.swaitcnt == 0) || (ref_rpdq.swaitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで送信待ちとなっているタスクID，データ情報を取得
	 */
	i = 1;
	p_next = &(p_pdqcb->swait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);
	*p_data = ((WINFO_SPDQ *)(((TCB *)(p_next->p_next))->p_winfo))->data;
	*p_datapri = ((WINFO_SPDQ *)(((TCB *)(p_next->p_next))->p_winfo))->datapri;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  優先度データキューの受信待ちタスク参照関数
 */
ER
ttsp_ref_rwait_pdq(ID pdqid, uint_t order, ID *p_tskid)
{
	PDQCB		*p_pdqcb;
	QUEUE		*p_next;
	ER			ercd;
	uint_t		i;
	bool_t		locked;
	T_TTSP_RPDQ	ref_rpdq;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_PDQID(pdqid));
	p_pdqcb = &_kernel_pdqcb_table[pdqid - 1];

	/*
	 *  受信待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_pdq(pdqid, &ref_rpdq);
	check_ercd(ercd, E_OK);
	if ((ref_rpdq.rwaitcnt == 0) || (ref_rpdq.rwaitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで受信待ちとなっているタスクIDを取得
	 */
	i = 1;
	p_next = &(p_pdqcb->rwait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


#if 0
/*
 *  ref_mbx代替関数
 */
ER
ttsp_ref_mbx(ID mbxid, T_TTSP_RMBX *pk_rmbx)
{
	MBXCB	*p_mbxcb;
	ER		ercd;
	bool_t	locked;
	uint_t	msgcnt;
	uint_t	waitcnt;
	QUEUE	*p_next;
	T_MSG	*pk_msg;

	locked = ttsp_loc_cpu();

	CHECK_MBXID(mbxid);
	p_mbxcb = &_kernel_mbxcb_table[mbxid - 1];

	/*
	 *  メールボックスにつながれているメッセージの数
	 */
	msgcnt = 0;
	pk_msg = p_mbxcb->pk_head;
	while (pk_msg != NULL) {
		msgcnt++;
		pk_msg = pk_msg->pk_next;
	}
	pk_rmbx->msgcnt = msgcnt;

	/*
	 *  メールボックス属性，メッセージ優先度の最大値を取得
	 */
	pk_rmbx->mbxatr = p_mbxcb->p_mbxinib->mbxatr;
	pk_rmbx->maxmpri = p_mbxcb->p_mbxinib->maxmpri;

	/*
	 *  受信待ちタスクの数
	 */
	waitcnt = 0;
	if (wait_tskid(&(p_mbxcb->wait_queue)) != TSK_NONE) {
		p_next = p_mbxcb->wait_queue.p_next;
		while (&(p_mbxcb->wait_queue) != p_next) {
			waitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rmbx->rwaitcnt = waitcnt;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  メールボックスにつながれているデータ参照関数
 */
ER
ttsp_ref_msg(ID mbxid, uint_t index, T_MSG **pp_msg)
{
	MBXCB		*p_mbxcb;
	ER			ercd;
	bool_t		locked;
	T_TTSP_RMBX	ref_rmbx;
	T_MSG		*pk_next;
	uint_t		i;

	locked = ttsp_loc_cpu();

	CHECK_MBXID(mbxid);
	p_mbxcb = &_kernel_mbxcb_table[mbxid - 1];

	/*
	 *  メールボックスにつながれているメッセージの数をチェック
	 */
	ercd = ttsp_ref_mbx(mbxid, &ref_rmbx);
	check_ercd(ercd, E_OK);
	if ((ref_rmbx.msgcnt == 0) || (ref_rmbx.msgcnt < index)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  indexに格納されているメッセージヘッダポインタを取得
	 */
	i = 1;
	pk_next = p_mbxcb->pk_head;
	while (i < index) {
		pk_next = pk_next->pk_next;
		i++;
	}
	*pp_msg = pk_next;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  メールボックスの受信待ちタスク参照関数
 */
ER
ttsp_ref_rwait_mbx(ID mbxid, uint_t order, ID *p_tskid)
{
	MBXCB		*p_mbxcb;
	ER			ercd;
	bool_t		locked;
	T_TTSP_RMBX	ref_rmbx;
	QUEUE		*p_next;
	uint_t		i;

	locked = ttsp_loc_cpu();

	CHECK_MBXID(mbxid);
	p_mbxcb = &_kernel_mbxcb_table[mbxid - 1];

	/*
	 *  受信待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_mbx(mbxid, &ref_rmbx);
	check_ercd(ercd, E_OK);
	if ((ref_rmbx.rwaitcnt == 0) || (ref_rmbx.rwaitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで受信待ちとなっているタスクIDを取得
	 */
	i = 1;
	p_next = &(p_mbxcb->wait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}
	*p_tskid = wait_tskid(p_next);

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}
#endif

/*
 *  ref_mpf代替関数
 */
ER
ttsp_ref_mpf(ID mpfid, T_TTSP_RMPF *pk_rmpf)
{
	MPFCB	*p_mpfcb;
	ER		ercd;
	bool_t	locked;
	uint_t	waitcnt;
	QUEUE	*p_next;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_MPFID(mpfid));
	p_mpfcb = &_kernel_mpfcb_table[mpfid - 1];

	/*
	 *  固定長メモリプール領域の空きメモリ領域に割り付けることができる
	 *  固定長メモリブロックの数，固定長メモリプール属性，メモリブロック数，
	 *  メモリブロックのサイズを取得
	 */
	pk_rmpf->fblkcnt = p_mpfcb->fblkcnt;
	pk_rmpf->mpfatr = p_mpfcb->p_mpfinib->mpfatr;
	pk_rmpf->blkcnt = p_mpfcb->p_mpfinib->blkcnt;
	pk_rmpf->blksz = p_mpfcb->p_mpfinib->blksz;
	pk_rmpf->mpf = p_mpfcb->p_mpfinib->mpf;
	
	/*
	 *  待ちタスクの数
	 */
	waitcnt = 0;
	if (wait_tskid(&(p_mpfcb->wait_queue)) != TSK_NONE) {
		p_next = p_mpfcb->wait_queue.p_next;
		while (&(p_mpfcb->wait_queue) != p_next) {
			waitcnt++;
			p_next = p_next->p_next;
		}
	}
	pk_rmpf->waitcnt =waitcnt;
	
	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  固定長メモリプールの待ちタスク参照関数
 */
ER
ttsp_ref_wait_mpf(ID mpfid, uint_t order, ID *p_tskid)
{
	MPFCB		*p_mpfcb;
	ER			ercd;
	bool_t		locked;
	T_TTSP_RMPF	ref_rmpf;
	QUEUE		*p_next;
	uint_t		i;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_MPFID(mpfid));
	p_mpfcb = &_kernel_mpfcb_table[mpfid - 1];

	/*
	 *  待ちタスクの数をチェック
	 */
	ercd = ttsp_ref_mpf(mpfid, &ref_rmpf);
	check_ercd(ercd, E_OK);
	if ((ref_rmpf.waitcnt == 0) || (ref_rmpf.waitcnt < order)) {
		ercd = E_PAR;
		goto error_exit;
	}

	/*
	 *  orderで待ちとなっているタスクIDを取得
	 */
	i = 1;
	p_next = &(p_mpfcb->wait_queue);
	while (i < order) {
		p_next = p_next->p_next;
		i++;
	}

	*p_tskid = wait_tskid(p_next);

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  ref_cyc代替関数
 */
ER
ttsp_ref_cyc(ID cycid, T_TTSP_RCYC *pk_rcyc)
{
	CYCCB	*p_cyccb;
	ER		ercd;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_CYCID(cycid));
	p_cyccb = &_kernel_cyccb_table[cycid - 1];

	/*
	 *  次に周期ハンドラを起動する時刻までの相対時間
	 */
	if (p_cyccb->cycsta) {
		pk_rcyc->cycstat = TCYC_STA;
		pk_rcyc->lefttim = tmevt_lefttim(&(p_cyccb->tmevtb));
	}
	else {
		pk_rcyc->cycstat = TCYC_STP;
	}

	/*
	 *  周期ハンドラ属性，周期ハンドラの拡張情報，
	 *  周期ハンドラの起動周期，周期ハンドラの起動位相を取得
	 */
	pk_rcyc->cycatr = p_cyccb->p_cycinib->cycatr;
	pk_rcyc->exinf = p_cyccb->p_cycinib->exinf;
	pk_rcyc->cyctim = p_cyccb->p_cycinib->cyctim;
	pk_rcyc->cycphs = p_cyccb->p_cycinib->cycphs;

	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  ref_alm代替関数
 */
ER
ttsp_ref_alm(ID almid, T_TTSP_RALM *pk_ralm)
{
	ALMCB	*p_almcb;
	ER		ercd;
	bool_t	locked;

	locked = ttsp_loc_cpu();

	CHECK_ID(VALID_ALMID(almid));
	p_almcb = &_kernel_almcb_table[almid - 1];

	/*
	 *  アラームハンドラを起動する時刻までの相対時間
	 */
	if (p_almcb->almsta) {
		pk_ralm->almstat = TALM_STA;
		pk_ralm->lefttim = tmevt_lefttim(&(p_almcb->tmevtb));
	}
	else {
		pk_ralm->almstat = TALM_STP;
	}

	/*
	 *  アラームハンドラ属性，アラームハンドラの拡張情報を取得
	 */
	pk_ralm->almatr = p_almcb->p_alminib->almatr;
	pk_ralm->exinf = p_almcb->p_alminib->exinf;
	
	ercd = E_OK;

  error_exit:
	ttsp_unl_cpu(locked);
	return(ercd);
}


/*
 *  get_ipm代替関数
 */
ER
ttsp_get_ipm(PRI *p_intpri)
{
	bool_t	locked;

	locked = ttsp_loc_cpu();

	/*
	 *  割込み優先度マスクを取得
	 */
	if (!sense_context()) {
		*p_intpri = t_get_ipm();
	}
	else {
		*p_intpri = i_get_ipm();
	}

	ttsp_unl_cpu(locked);

	return(E_OK);
}
