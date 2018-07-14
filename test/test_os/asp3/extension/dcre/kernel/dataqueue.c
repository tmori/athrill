/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2017 by Embedded and Real-Time Systems Laboratory
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
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: dataqueue.c 781 2017-03-10 23:31:33Z ertl-hiro $
 */

/*
 *		データキュー機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "wait.h"
#include "dataqueue.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_ACRE_DTQ_ENTER
#define LOG_ACRE_DTQ_ENTER(pk_cdtq)
#endif /* LOG_ACRE_DTQ_ENTER */

#ifndef LOG_ACRE_DTQ_LEAVE
#define LOG_ACRE_DTQ_LEAVE(ercd)
#endif /* LOG_ACRE_DTQ_LEAVE */

#ifndef LOG_DEL_DTQ_ENTER
#define LOG_DEL_DTQ_ENTER(dtqid)
#endif /* LOG_DEL_DTQ_ENTER */

#ifndef LOG_DEL_DTQ_LEAVE
#define LOG_DEL_DTQ_LEAVE(ercd)
#endif /* LOG_DEL_DTQ_LEAVE */

#ifndef LOG_SND_DTQ_ENTER
#define LOG_SND_DTQ_ENTER(dtqid, data)
#endif /* LOG_SND_DTQ_ENTER */

#ifndef LOG_SND_DTQ_LEAVE
#define LOG_SND_DTQ_LEAVE(ercd)
#endif /* LOG_SND_DTQ_LEAVE */

#ifndef LOG_PSND_DTQ_ENTER
#define LOG_PSND_DTQ_ENTER(dtqid, data)
#endif /* LOG_PSND_DTQ_ENTER */

#ifndef LOG_PSND_DTQ_LEAVE
#define LOG_PSND_DTQ_LEAVE(ercd)
#endif /* LOG_PSND_DTQ_LEAVE */

#ifndef LOG_TSND_DTQ_ENTER
#define LOG_TSND_DTQ_ENTER(dtqid, data, tmout)
#endif /* LOG_TSND_DTQ_ENTER */

#ifndef LOG_TSND_DTQ_LEAVE
#define LOG_TSND_DTQ_LEAVE(ercd)
#endif /* LOG_TSND_DTQ_LEAVE */

#ifndef LOG_FSND_DTQ_ENTER
#define LOG_FSND_DTQ_ENTER(dtqid, data)
#endif /* LOG_FSND_DTQ_ENTER */

#ifndef LOG_FSND_DTQ_LEAVE
#define LOG_FSND_DTQ_LEAVE(ercd)
#endif /* LOG_FSND_DTQ_LEAVE */

#ifndef LOG_RCV_DTQ_ENTER
#define LOG_RCV_DTQ_ENTER(dtqid, p_data)
#endif /* LOG_RCV_DTQ_ENTER */

#ifndef LOG_RCV_DTQ_LEAVE
#define LOG_RCV_DTQ_LEAVE(ercd, p_data)
#endif /* LOG_RCV_DTQ_LEAVE */

#ifndef LOG_PRCV_DTQ_ENTER
#define LOG_PRCV_DTQ_ENTER(dtqid, p_data)
#endif /* LOG_PRCV_DTQ_ENTER */

#ifndef LOG_PRCV_DTQ_LEAVE
#define LOG_PRCV_DTQ_LEAVE(ercd, p_data)
#endif /* LOG_PRCV_DTQ_LEAVE */

#ifndef LOG_TRCV_DTQ_ENTER
#define LOG_TRCV_DTQ_ENTER(dtqid, p_data, tmout)
#endif /* LOG_TRCV_DTQ_ENTER */

#ifndef LOG_TRCV_DTQ_LEAVE
#define LOG_TRCV_DTQ_LEAVE(ercd, p_data)
#endif /* LOG_TRCV_DTQ_LEAVE */

#ifndef LOG_INI_DTQ_ENTER
#define LOG_INI_DTQ_ENTER(dtqid)
#endif /* LOG_INI_DTQ_ENTER */

#ifndef LOG_INI_DTQ_LEAVE
#define LOG_INI_DTQ_LEAVE(ercd)
#endif /* LOG_INI_DTQ_LEAVE */

#ifndef LOG_REF_DTQ_ENTER
#define LOG_REF_DTQ_ENTER(dtqid, pk_rdtq)
#endif /* LOG_REF_DTQ_ENTER */

#ifndef LOG_REF_DTQ_LEAVE
#define LOG_REF_DTQ_LEAVE(ercd, pk_rdtq)
#endif /* LOG_REF_DTQ_LEAVE */

/*
 *  データキューの数
 */
#define tnum_dtq	((uint_t)(tmax_dtqid - TMIN_DTQID + 1))
#define tnum_sdtq	((uint_t)(tmax_sdtqid - TMIN_DTQID + 1))

/*
 *  データキューIDからデータキュー管理ブロックを取り出すためのマクロ
 */
#define INDEX_DTQ(dtqid)	((uint_t)((dtqid) - TMIN_DTQID))
#define get_dtqcb(dtqid)	(&(dtqcb_table[INDEX_DTQ(dtqid)]))

#ifdef TOPPERS_dtqini

/*
 *  使用していないデータキュー管理ブロックのリスト
 */
QUEUE	free_dtqcb;

/*
 *  データキュー機能の初期化
 */
void
initialize_dataqueue(void)
{
	uint_t	i, j;
	DTQCB	*p_dtqcb;
	DTQINIB	*p_dtqinib;

	for (i = 0; i < tnum_sdtq; i++) {
		p_dtqcb = &(dtqcb_table[i]);
		queue_initialize(&(p_dtqcb->swait_queue));
		p_dtqcb->p_dtqinib = &(dtqinib_table[i]);
		queue_initialize(&(p_dtqcb->rwait_queue));
		p_dtqcb->count = 0U;
		p_dtqcb->head = 0U;
		p_dtqcb->tail = 0U;
	}
	queue_initialize(&free_dtqcb);
	for (j = 0; i < tnum_dtq; i++, j++) {
		p_dtqcb = &(dtqcb_table[i]);
		p_dtqinib = &(adtqinib_table[j]);
		p_dtqinib->dtqatr = TA_NOEXS;
		p_dtqcb->p_dtqinib = ((const DTQINIB *) p_dtqinib);
		queue_insert_prev(&free_dtqcb, &(p_dtqcb->swait_queue));
	}
}

#endif /* TOPPERS_dtqini */

/*
 *  データキュー管理領域へのデータの格納
 */
#ifdef TOPPERS_dtqenq

void
enqueue_data(DTQCB *p_dtqcb, intptr_t data)
{
	(p_dtqcb->p_dtqinib->p_dtqmb + p_dtqcb->tail)->data = data;
	p_dtqcb->count++;
	p_dtqcb->tail++;
	if (p_dtqcb->tail >= p_dtqcb->p_dtqinib->dtqcnt) {
		p_dtqcb->tail = 0U;
	}
}

#endif /* TOPPERS_dtqenq */

/*
 *  データキュー管理領域へのデータの強制格納
 */
#ifdef TOPPERS_dtqfenq

void
force_enqueue_data(DTQCB *p_dtqcb, intptr_t data)
{
	(p_dtqcb->p_dtqinib->p_dtqmb + p_dtqcb->tail)->data = data;
	p_dtqcb->tail++;
	if (p_dtqcb->tail >= p_dtqcb->p_dtqinib->dtqcnt) {
		p_dtqcb->tail = 0U;
	}
	if (p_dtqcb->count < p_dtqcb->p_dtqinib->dtqcnt) {
		p_dtqcb->count++;
	}
	else {
		p_dtqcb->head = p_dtqcb->tail;
	}
}

#endif /* TOPPERS_dtqfenq */

/*
 *  データキュー管理領域からのデータの取出し
 */
#ifdef TOPPERS_dtqdeq

void
dequeue_data(DTQCB *p_dtqcb, intptr_t *p_data)
{
	*p_data = (p_dtqcb->p_dtqinib->p_dtqmb + p_dtqcb->head)->data;
	p_dtqcb->count--;
	p_dtqcb->head++;
	if (p_dtqcb->head >= p_dtqcb->p_dtqinib->dtqcnt) {
		p_dtqcb->head = 0U;
	}
}

#endif /* TOPPERS_dtqdeq */

/*
 *  データキューへのデータ送信
 */
#ifdef TOPPERS_dtqsnd

bool_t
send_data(DTQCB *p_dtqcb, intptr_t data)
{
	TCB		*p_tcb;

	if (!queue_empty(&(p_dtqcb->rwait_queue))) {
		p_tcb = (TCB *) queue_delete_next(&(p_dtqcb->rwait_queue));
		((WINFO_RDTQ *)(p_tcb->p_winfo))->data = data;
		wait_complete(p_tcb);
		return(true);
	}
	else if (p_dtqcb->count < p_dtqcb->p_dtqinib->dtqcnt) {
		enqueue_data(p_dtqcb, data);
		return(true);
	}
	else {
		return(false);
	}
}

#endif /* TOPPERS_dtqsnd */

/*
 *  データキューへのデータ強制送信
 */
#ifdef TOPPERS_dtqfsnd

void
force_send_data(DTQCB *p_dtqcb, intptr_t data)
{
	TCB		*p_tcb;

	if (!queue_empty(&(p_dtqcb->rwait_queue))) {
		p_tcb = (TCB *) queue_delete_next(&(p_dtqcb->rwait_queue));
		((WINFO_RDTQ *)(p_tcb->p_winfo))->data = data;
		wait_complete(p_tcb);
	}
	else {
		force_enqueue_data(p_dtqcb, data);
	}
}

#endif /* TOPPERS_dtqfsnd */

/*
 *  データキューからのデータ受信
 */
#ifdef TOPPERS_dtqrcv

bool_t
receive_data(DTQCB *p_dtqcb, intptr_t *p_data)
{
	TCB		*p_tcb;
	intptr_t data;

	if (p_dtqcb->count > 0U) {
		dequeue_data(p_dtqcb, p_data);
		if (!queue_empty(&(p_dtqcb->swait_queue))) {
			p_tcb = (TCB *) queue_delete_next(&(p_dtqcb->swait_queue));
			data = ((WINFO_SDTQ *)(p_tcb->p_winfo))->data;
			enqueue_data(p_dtqcb, data);
			wait_complete(p_tcb);
		}
		return(true);
	}
	else if (!queue_empty(&(p_dtqcb->swait_queue))) {
		p_tcb = (TCB *) queue_delete_next(&(p_dtqcb->swait_queue));
		*p_data = ((WINFO_SDTQ *)(p_tcb->p_winfo))->data;
		wait_complete(p_tcb);
		return(true);
	}
	else {
		return(false);
	}
}

#endif /* TOPPERS_dtqrcv */

/*
 *  データキューの生成
 */
#ifdef TOPPERS_acre_dtq

ER_UINT
acre_dtq(const T_CDTQ *pk_cdtq)
{
	DTQCB		*p_dtqcb;
	DTQINIB		*p_dtqinib;
	ATR			dtqatr;
	uint_t		dtqcnt;
	DTQMB		*p_dtqmb;
	ER			ercd;

	LOG_ACRE_DTQ_ENTER(pk_cdtq);
	CHECK_TSKCTX_UNL();

	dtqatr = pk_cdtq->dtqatr;
	dtqcnt = pk_cdtq->dtqcnt;
	p_dtqmb = pk_cdtq->dtqmb;

	CHECK_RSATR(dtqatr, TA_TPRI);

	lock_cpu();
	if (tnum_dtq == 0 || queue_empty(&free_dtqcb)) {
		ercd = E_NOID;
	}
	else {
		if (dtqcnt != 0 && p_dtqmb == NULL) {
			p_dtqmb = kernel_malloc(sizeof(DTQMB) * dtqcnt);
			dtqatr |= TA_MBALLOC;
		}
		if (dtqcnt != 0 && p_dtqmb == NULL) {
			ercd = E_NOMEM;
		}
		else {
			p_dtqcb = ((DTQCB *) queue_delete_next(&free_dtqcb));
			p_dtqinib = (DTQINIB *)(p_dtqcb->p_dtqinib);
			p_dtqinib->dtqatr = dtqatr;
			p_dtqinib->dtqcnt = dtqcnt;
			p_dtqinib->p_dtqmb = p_dtqmb;

			queue_initialize(&(p_dtqcb->swait_queue));
			queue_initialize(&(p_dtqcb->rwait_queue));
			p_dtqcb->count = 0U;
			p_dtqcb->head = 0U;
			p_dtqcb->tail = 0U;
			ercd = DTQID(p_dtqcb);
		}
	}
	unlock_cpu();

  error_exit:
	LOG_ACRE_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_acre_dtq */

/*
 *  データキューの削除
 */
#ifdef TOPPERS_del_dtq

ER
del_dtq(ID dtqid)
{
	DTQCB	*p_dtqcb;
	DTQINIB	*p_dtqinib;
	ER		ercd;

	LOG_DEL_DTQ_ENTER(dtqid);
	CHECK_TSKCTX_UNL();
	CHECK_PAR(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (dtqid <= tmax_sdtqid) {
		ercd = E_OBJ;
	}
	else {
		init_wait_queue(&(p_dtqcb->swait_queue));
		init_wait_queue(&(p_dtqcb->rwait_queue));
		p_dtqinib = (DTQINIB *)(p_dtqcb->p_dtqinib);
		if ((p_dtqinib->dtqatr & TA_MBALLOC) != 0U) {
			kernel_free(p_dtqinib->p_dtqmb);
		}
		p_dtqinib->dtqatr = TA_NOEXS;
		queue_insert_prev(&free_dtqcb, &(p_dtqcb->swait_queue));
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_DEL_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_del_dtq */

/*
 *  データキューへの送信
 */
#ifdef TOPPERS_snd_dtq

ER
snd_dtq(ID dtqid, intptr_t data)
{
	DTQCB	*p_dtqcb;
	WINFO_SDTQ winfo_sdtq;
	ER		ercd;

	LOG_SND_DTQ_ENTER(dtqid, data);
	CHECK_DISPATCH();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu_dsp();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_runtsk->raster) {
		ercd = E_RASTER;
	}
	else if (send_data(p_dtqcb, data)) {
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	else {
		winfo_sdtq.data = data;
		p_runtsk->tstat = TS_WAITING_SDTQ;
		wobj_make_wait((WOBJCB *) p_dtqcb, (WINFO_WOBJ *) &winfo_sdtq);
		dispatch();
		ercd = winfo_sdtq.winfo.wercd;
	}
	unlock_cpu_dsp();

  error_exit:
	LOG_SND_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_snd_dtq */

/*
 *  データキューへの送信（ポーリング）
 */
#ifdef TOPPERS_psnd_dtq

ER
psnd_dtq(ID dtqid, intptr_t data)
{
	DTQCB	*p_dtqcb;
	ER		ercd;

	LOG_PSND_DTQ_ENTER(dtqid, data);
	CHECK_UNL();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (send_data(p_dtqcb, data)) {
		if (p_runtsk != p_schedtsk) {
			if (!sense_context()) {
				dispatch();
			}
			else {
				request_dispatch();
			}
		}
		ercd = E_OK;
	}
	else {
		ercd = E_TMOUT;
	}
	unlock_cpu();

  error_exit:
	LOG_PSND_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_psnd_dtq */

/*
 *  データキューへの送信（タイムアウトあり）
 */
#ifdef TOPPERS_tsnd_dtq

ER
tsnd_dtq(ID dtqid, intptr_t data, TMO tmout)
{
	DTQCB	*p_dtqcb;
	WINFO_SDTQ winfo_sdtq;
	TMEVTB	tmevtb;
	ER		ercd;

	LOG_TSND_DTQ_ENTER(dtqid, data, tmout);
	CHECK_DISPATCH();
	CHECK_ID(VALID_DTQID(dtqid));
	CHECK_PAR(VALID_TMOUT(tmout));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu_dsp();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_runtsk->raster) {
		ercd = E_RASTER;
	}
	else if (send_data(p_dtqcb, data)) {
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	else if (tmout == TMO_POL) {
		ercd = E_TMOUT;
	}
	else {
		winfo_sdtq.data = data;
		p_runtsk->tstat = TS_WAITING_SDTQ;
		wobj_make_wait_tmout((WOBJCB *) p_dtqcb, (WINFO_WOBJ *) &winfo_sdtq,
														&tmevtb, tmout);
		dispatch();
		ercd = winfo_sdtq.winfo.wercd;
	}
	unlock_cpu_dsp();

  error_exit:
	LOG_TSND_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_tsnd_dtq */

/*
 *  データキューへの強制送信
 */
#ifdef TOPPERS_fsnd_dtq

ER
fsnd_dtq(ID dtqid, intptr_t data)
{
	DTQCB	*p_dtqcb;	
	ER		ercd;

	LOG_FSND_DTQ_ENTER(dtqid, data);
	CHECK_UNL();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (!(p_dtqcb->p_dtqinib->dtqcnt > 0U)) {
		ercd = E_ILUSE;
	}
	else {
		force_send_data(p_dtqcb, data);
		if (p_runtsk != p_schedtsk) {
			if (!sense_context()) {
				dispatch();
			}
			else {
				request_dispatch();
			}
		}
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_FSND_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_fsnd_dtq */

/*
 *  データキューからの受信
 */
#ifdef TOPPERS_rcv_dtq

ER
rcv_dtq(ID dtqid, intptr_t *p_data)
{
	DTQCB	*p_dtqcb;
	WINFO_RDTQ winfo_rdtq;
	ER		ercd;

	LOG_RCV_DTQ_ENTER(dtqid, p_data);
	CHECK_DISPATCH();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu_dsp();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_runtsk->raster) {
		ercd = E_RASTER;
	}
	else if (receive_data(p_dtqcb, p_data)) {
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	else {
		p_runtsk->tstat = TS_WAITING_RDTQ;
		make_wait(&(winfo_rdtq.winfo));
		queue_insert_prev(&(p_dtqcb->rwait_queue), &(p_runtsk->task_queue));
		winfo_rdtq.p_dtqcb = p_dtqcb;
		LOG_TSKSTAT(p_runtsk);
		dispatch();
		ercd = winfo_rdtq.winfo.wercd;
		if (ercd == E_OK) {
			*p_data = winfo_rdtq.data;
		}
	}
	unlock_cpu_dsp();

  error_exit:
	LOG_RCV_DTQ_LEAVE(ercd, p_data);
	return(ercd);
}

#endif /* TOPPERS_rcv_dtq */

/*
 *  データキューからの受信（ポーリング）
 */
#ifdef TOPPERS_prcv_dtq

ER
prcv_dtq(ID dtqid, intptr_t *p_data)
{
	DTQCB	*p_dtqcb;
	ER		ercd;

	LOG_PRCV_DTQ_ENTER(dtqid, p_data);
	CHECK_TSKCTX_UNL();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (receive_data(p_dtqcb, p_data)) {
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	else {
		ercd = E_TMOUT;
	}
	unlock_cpu();

  error_exit:
	LOG_PRCV_DTQ_LEAVE(ercd, p_data);
	return(ercd);
}

#endif /* TOPPERS_prcv_dtq */

/*
 *  データキューからの受信（タイムアウトあり）
 */
#ifdef TOPPERS_trcv_dtq

ER
trcv_dtq(ID dtqid, intptr_t *p_data, TMO tmout)
{
	DTQCB	*p_dtqcb;
	WINFO_RDTQ winfo_rdtq;
	TMEVTB	tmevtb;
	ER		ercd;

	LOG_TRCV_DTQ_ENTER(dtqid, p_data, tmout);
	CHECK_DISPATCH();
	CHECK_ID(VALID_DTQID(dtqid));
	CHECK_PAR(VALID_TMOUT(tmout));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu_dsp();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else if (p_runtsk->raster) {
		ercd = E_RASTER;
	}
	else if (receive_data(p_dtqcb, p_data)) {
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	else if (tmout == TMO_POL) {
		ercd = E_TMOUT;
	}
	else {
		p_runtsk->tstat = TS_WAITING_RDTQ;
		make_wait_tmout(&(winfo_rdtq.winfo), &tmevtb, tmout);
		queue_insert_prev(&(p_dtqcb->rwait_queue), &(p_runtsk->task_queue));
		winfo_rdtq.p_dtqcb = p_dtqcb;
		LOG_TSKSTAT(p_runtsk);
		dispatch();
		ercd = winfo_rdtq.winfo.wercd;
		if (ercd == E_OK) {
			*p_data = winfo_rdtq.data;
		}
	}
	unlock_cpu_dsp();

  error_exit:
	LOG_TRCV_DTQ_LEAVE(ercd, p_data);
	return(ercd);
}

#endif /* TOPPERS_trcv_dtq */

/*
 *  データキューの再初期化
 */
#ifdef TOPPERS_ini_dtq

ER
ini_dtq(ID dtqid)
{
	DTQCB	*p_dtqcb;
	ER		ercd;
    
	LOG_INI_DTQ_ENTER(dtqid);
	CHECK_TSKCTX_UNL();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		init_wait_queue(&(p_dtqcb->swait_queue));
		init_wait_queue(&(p_dtqcb->rwait_queue));
		p_dtqcb->count = 0U;
		p_dtqcb->head = 0U;
		p_dtqcb->tail = 0U;
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_INI_DTQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ini_dtq */

/*
 *  データキューの状態参照
 */
#ifdef TOPPERS_ref_dtq

ER
ref_dtq(ID dtqid, T_RDTQ *pk_rdtq)
{
	DTQCB	*p_dtqcb;
	ER		ercd;
    
	LOG_REF_DTQ_ENTER(dtqid, pk_rdtq);
	CHECK_TSKCTX_UNL();
	CHECK_ID(VALID_DTQID(dtqid));
	p_dtqcb = get_dtqcb(dtqid);

	lock_cpu();
	if (p_dtqcb->p_dtqinib->dtqatr == TA_NOEXS) {
		ercd = E_NOEXS;
	}
	else {
		pk_rdtq->stskid = wait_tskid(&(p_dtqcb->swait_queue));
		pk_rdtq->rtskid = wait_tskid(&(p_dtqcb->rwait_queue));
		pk_rdtq->sdtqcnt = p_dtqcb->count;
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_REF_DTQ_LEAVE(ercd, pk_rdtq);
	return(ercd);
}

#endif /* TOPPERS_ref_dtq */
