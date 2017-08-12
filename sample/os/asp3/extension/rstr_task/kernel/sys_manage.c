/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2015 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: sys_manage.c 684 2016-03-11 15:24:12Z ertl-hiro $
 */

/*
 *		システム状態管理機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_ROT_RDQ_ENTER
#define LOG_ROT_RDQ_ENTER(tskpri)
#endif /* LOG_ROT_RDQ_ENTER */

#ifndef LOG_ROT_RDQ_LEAVE
#define LOG_ROT_RDQ_LEAVE(ercd)
#endif /* LOG_ROT_RDQ_LEAVE */

#ifndef LOG_GET_TID_ENTER
#define LOG_GET_TID_ENTER(p_tskid)
#endif /* LOG_GET_TID_ENTER */

#ifndef LOG_GET_TID_LEAVE
#define LOG_GET_TID_LEAVE(ercd, p_tskid)
#endif /* LOG_GET_TID_LEAVE */

#ifndef LOG_GET_LOD_ENTER
#define LOG_GET_LOD_ENTER(tskpri, p_load)
#endif /* LOG_GET_LOD_ENTER */

#ifndef LOG_GET_LOD_LEAVE
#define LOG_GET_LOD_LEAVE(ercd, p_load)
#endif /* LOG_GET_LOD_LEAVE */

#ifndef LOG_GET_NTH_ENTER
#define LOG_GET_NTH_ENTER(tskpri, nth, p_tskid)
#endif /* LOG_GET_NTH_ENTER */

#ifndef LOG_GET_NTH_LEAVE
#define LOG_GET_NTH_LEAVE(ercd, p_tskid)
#endif /* LOG_GET_NTH_LEAVE */

#ifndef LOG_LOC_CPU_ENTER
#define LOG_LOC_CPU_ENTER()
#endif /* LOG_LOC_CPU_ENTER */

#ifndef LOG_LOC_CPU_LEAVE
#define LOG_LOC_CPU_LEAVE(ercd)
#endif /* LOG_LOC_CPU_LEAVE */

#ifndef LOG_UNL_CPU_ENTER
#define LOG_UNL_CPU_ENTER()
#endif /* LOG_UNL_CPU_ENTER */

#ifndef LOG_UNL_CPU_LEAVE
#define LOG_UNL_CPU_LEAVE(ercd)
#endif /* LOG_UNL_CPU_LEAVE */

#ifndef LOG_DIS_DSP_ENTER
#define LOG_DIS_DSP_ENTER()
#endif /* LOG_DIS_DSP_ENTER */

#ifndef LOG_DIS_DSP_LEAVE
#define LOG_DIS_DSP_LEAVE(ercd)
#endif /* LOG_DIS_DSP_LEAVE */

#ifndef LOG_ENA_DSP_ENTER
#define LOG_ENA_DSP_ENTER()
#endif /* LOG_ENA_DSP_ENTER */

#ifndef LOG_ENA_DSP_LEAVE
#define LOG_ENA_DSP_LEAVE(ercd)
#endif /* LOG_ENA_DSP_LEAVE */

#ifndef LOG_SNS_CTX_ENTER
#define LOG_SNS_CTX_ENTER()
#endif /* LOG_SNS_CTX_ENTER */

#ifndef LOG_SNS_CTX_LEAVE
#define LOG_SNS_CTX_LEAVE(state)
#endif /* LOG_SNS_CTX_LEAVE */

#ifndef LOG_SNS_LOC_ENTER
#define LOG_SNS_LOC_ENTER()
#endif /* LOG_SNS_LOC_ENTER */

#ifndef LOG_SNS_LOC_LEAVE
#define LOG_SNS_LOC_LEAVE(state)
#endif /* LOG_SNS_LOC_LEAVE */

#ifndef LOG_SNS_DSP_ENTER
#define LOG_SNS_DSP_ENTER()
#endif /* LOG_SNS_DSP_ENTER */

#ifndef LOG_SNS_DSP_LEAVE
#define LOG_SNS_DSP_LEAVE(state)
#endif /* LOG_SNS_DSP_LEAVE */

#ifndef LOG_SNS_DPN_ENTER
#define LOG_SNS_DPN_ENTER()
#endif /* LOG_SNS_DPN_ENTER */

#ifndef LOG_SNS_DPN_LEAVE
#define LOG_SNS_DPN_LEAVE(state)
#endif /* LOG_SNS_DPN_LEAVE */

#ifndef LOG_SNS_KER_ENTER
#define LOG_SNS_KER_ENTER()
#endif /* LOG_SNS_KER_ENTER */

#ifndef LOG_SNS_KER_LEAVE
#define LOG_SNS_KER_LEAVE(state)
#endif /* LOG_SNS_KER_LEAVE */

/*
 *  タスクの優先順位の回転［NGKI3548］
 */
#ifdef TOPPERS_rot_rdq

ER
rot_rdq(PRI tskpri)
{
	uint_t	pri;
	QUEUE	*p_queue;
	ER		ercd;

	LOG_ROT_RDQ_ENTER(tskpri);
	CHECK_UNL();								/*［NGKI2684］*/
	if (tskpri == TPRI_SELF && !sense_context()) {
		pri = p_runtsk->bpriority;				/*［NGKI2689］*/
	}
	else {
		CHECK_PAR(VALID_TPRI(tskpri));			/*［NGKI2685］*/
		pri = INT_PRIORITY(tskpri);
	}

	lock_cpu();
	p_queue = &(ready_queue[pri]);
	if (queue_empty(p_queue)) {
		ercd = E_OK;
	}
	else if ((((TCB *)(p_queue->p_next))->p_tinib->tskatr & TA_RSTR) != 0U) {
		ercd = E_NOSPT;
	}
	else {
		rotate_ready_queue(&(ready_queue[pri]));
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
	LOG_ROT_RDQ_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_rot_rdq */

/*
 *  実行状態のタスクIDの参照［NGKI3550］
 */
#ifdef TOPPERS_get_tid

ER
get_tid(ID *p_tskid)
{
	ER		ercd;

	LOG_GET_TID_ENTER(p_tskid);
	CHECK_UNL();								/*［NGKI2707］*/

	lock_cpu();
	*p_tskid = (p_runtsk == NULL) ? TSK_NONE : TSKID(p_runtsk);
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_GET_TID_LEAVE(ercd, p_tskid);
	return(ercd);
}

#endif /* TOPPERS_get_tid */

/*
 *  実行できるタスクの数の参照［NGKI3623］
 */
#ifdef TOPPERS_get_lod

ER
get_lod(PRI tskpri, uint_t *p_load)
{
	uint_t	pri, load;
	QUEUE	*p_queue, *p_entry;
	ER		ercd;

	LOG_GET_LOD_ENTER(p_tskid, p_load);
	CHECK_TSKCTX_UNL();							/*［NGKI3624］［NGKI3625］*/
	if (tskpri == TPRI_SELF) {
		pri = p_runtsk->bpriority;				/*［NGKI3631］*/
	}
	else {
		CHECK_PAR(VALID_TPRI(tskpri));			/*［NGKI3626］*/
		pri = INT_PRIORITY(tskpri);
	}
	p_queue = &(ready_queue[pri]);

	lock_cpu();
	load = 0U;
	for (p_entry = p_queue->p_next; p_entry != p_queue;
										p_entry = p_entry->p_next) {
		load += 1U;
	}
	*p_load = load;
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_GET_LOD_LEAVE(ercd, p_load);
	return(ercd);
}

#endif /* TOPPERS_get_lod */

/*
 *  指定した優先順位のタスクIDの参照［NGKI3641］
 */
#ifdef TOPPERS_get_nth

ER
get_nth(PRI tskpri, uint_t nth, ID *p_tskid)
{
	uint_t	pri;
	QUEUE	*p_queue, *p_entry;
	ID		tskid;
	ER		ercd;

	LOG_GET_NTH_ENTER(p_tskid, nth, p_tskid);
	CHECK_TSKCTX_UNL();							/*［NGKI3642］［NGKI3643］*/
	if (tskpri == TPRI_SELF) {
		pri = p_runtsk->bpriority;				/*［NGKI3650］*/
	}
	else {
		CHECK_PAR(VALID_TPRI(tskpri));			/*［NGKI3644］*/
		pri = INT_PRIORITY(tskpri);
	}
	p_queue = &(ready_queue[pri]);

	lock_cpu();
	tskid = TSK_NONE;
	for (p_entry = p_queue->p_next; p_entry != p_queue;
										p_entry = p_entry->p_next) {
		if (nth == 0U) {
			tskid = TSKID((TCB *) p_entry);
			break;
		}
		nth -= 1U;
	}
	*p_tskid = tskid;
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_GET_NTH_LEAVE(ercd, p_tskid);
	return(ercd);
}

#endif /* TOPPERS_get_nth */

/*
 *  CPUロック状態への遷移［NGKI3538］
 */
#ifdef TOPPERS_loc_cpu

ER
loc_cpu(void)
{
	ER		ercd;

	LOG_LOC_CPU_ENTER();

	if (!sense_lock()) {						/*［NGKI2731］*/
		lock_cpu();								/*［NGKI2730］*/
	}
	ercd = E_OK;

	LOG_LOC_CPU_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_loc_cpu */

/*
 *  CPUロック状態の解除［NGKI3539］
 *
 *  CPUロック中は，ディスパッチが必要となるサービスコールを呼び出すこ
 *  とはできないため，CPUロック状態の解除時にディスパッチャを起動する
 *  必要はない．
 */
#ifdef TOPPERS_unl_cpu

ER
unl_cpu(void)
{
	ER		ercd;

	LOG_UNL_CPU_ENTER();

	if (sense_lock()) {							/*［NGKI2738］*/
		unlock_cpu();							/*［NGKI2737］*/
	}
	ercd = E_OK;

	LOG_UNL_CPU_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_unl_cpu */

/*
 *  ディスパッチの禁止［NGKI2740］
 */
#ifdef TOPPERS_dis_dsp

ER
dis_dsp(void)
{
	ER		ercd;

	LOG_DIS_DSP_ENTER();
	CHECK_TSKCTX_UNL();							/*［NGKI2741］［NGKI2742］*/

	lock_cpu();
	enadsp = false;
	dspflg = false;
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_DIS_DSP_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_dis_dsp */

/*
 *  ディスパッチの許可［NGKI2746］
 */
#ifdef TOPPERS_ena_dsp

ER
ena_dsp(void)
{
	ER		ercd;

	LOG_ENA_DSP_ENTER();
	CHECK_TSKCTX_UNL();							/*［NGKI2747］［NGKI2748］*/

	lock_cpu();
	enadsp = true;
	if (t_get_ipm() == TIPM_ENAALL) {
		dspflg = true;
		p_schedtsk = search_schedtsk();
		if (p_runtsk->raster && p_runtsk->enater) {
			task_terminate(p_runtsk);
			exit_and_dispatch();
			ercd = E_SYS;
		}
		else {
			if (p_runtsk != p_schedtsk) {
				dispatch();
			}
			ercd = E_OK;
		}
	}
	else {
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_ENA_DSP_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ena_dsp */

/*
 *  コンテキストの参照［NGKI2752］
 */
#ifdef TOPPERS_sns_ctx

bool_t
sns_ctx(void)
{
	bool_t	state;

	LOG_SNS_CTX_ENTER();
	state = sense_context() ? true : false;
	LOG_SNS_CTX_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_ctx */

/*
 *  CPUロック状態の参照［NGKI2754］
 */
#ifdef TOPPERS_sns_loc

bool_t
sns_loc(void)
{
	bool_t	state;

	LOG_SNS_LOC_ENTER();
	state = sense_lock() ? true : false;
	LOG_SNS_LOC_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_loc */

/*
 *  ディスパッチ禁止状態の参照［NGKI2756］
 */
#ifdef TOPPERS_sns_dsp

bool_t
sns_dsp(void)
{
	bool_t	state;

	LOG_SNS_DSP_ENTER();
	state = !enadsp;
	LOG_SNS_DSP_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_dsp */

/*
 *  ディスパッチ保留状態の参照［NGKI2758］
 */
#ifdef TOPPERS_sns_dpn

bool_t
sns_dpn(void)
{
	bool_t	state;

	LOG_SNS_DPN_ENTER();
	state = (sense_context() || sense_lock() || !dspflg) ? true : false;
	LOG_SNS_DPN_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_dpn */

/*
 *  カーネル非動作状態の参照［NGKI2760］
 */
#ifdef TOPPERS_sns_ker

bool_t
sns_ker(void)
{
	bool_t	state;

	LOG_SNS_KER_ENTER();
	state = kerflg ? false : true;
	LOG_SNS_KER_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_ker */
