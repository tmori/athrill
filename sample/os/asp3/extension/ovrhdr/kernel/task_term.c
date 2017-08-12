/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2005-2014 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: task_term.c 275 2014-11-03 14:26:05Z ertl-hiro $
 */

/*
 *		タスク終了機能
 */

#include "kernel_impl.h"
#include "check.h"
#include "task.h"
#include "wait.h"
#include "overrun.h"

/*
 *  トレースログマクロのデフォルト定義
 */
#ifndef LOG_EXT_TSK_ENTER
#define LOG_EXT_TSK_ENTER()
#endif /* LOG_EXT_TSK_ENTER */

#ifndef LOG_EXT_TSK_LEAVE
#define LOG_EXT_TSK_LEAVE(ercd)
#endif /* LOG_EXT_TSK_LEAVE */

#ifndef LOG_RAS_TER_ENTER
#define LOG_RAS_TER_ENTER(tskid)
#endif /* LOG_RAS_TER_ENTER */

#ifndef LOG_RAS_TER_LEAVE
#define LOG_RAS_TER_LEAVE(ercd)
#endif /* LOG_RAS_TER_LEAVE */

#ifndef LOG_DIS_TER_ENTER
#define LOG_DIS_TER_ENTER()
#endif /* LOG_DIS_TER_ENTER */

#ifndef LOG_DIS_TER_LEAVE
#define LOG_DIS_TER_LEAVE(ercd)
#endif /* LOG_DIS_TER_LEAVE */

#ifndef LOG_ENA_TER_ENTER
#define LOG_ENA_TER_ENTER()
#endif /* LOG_ENA_TER_ENTER */

#ifndef LOG_ENA_TER_LEAVE
#define LOG_ENA_TER_LEAVE(ercd)
#endif /* LOG_ENA_TER_LEAVE */

#ifndef LOG_SNS_TER_ENTER
#define LOG_SNS_TER_ENTER()
#endif /* LOG_SNS_TER_ENTER */

#ifndef LOG_SNS_TER_LEAVE
#define LOG_SNS_TER_LEAVE(state)
#endif /* LOG_SNS_TER_LEAVE */

#ifndef LOG_TER_TSK_ENTER
#define LOG_TER_TSK_ENTER(tskid)
#endif /* LOG_TER_TSK_ENTER */

#ifndef LOG_TER_TSK_LEAVE
#define LOG_TER_TSK_LEAVE(ercd)
#endif /* LOG_TER_TSK_LEAVE */

/*
 *  自タスクの終了［NGKI1162］
 */
#ifdef TOPPERS_ext_tsk

ER
ext_tsk(void)
{
	PRI		intpri;
	ER		ercd;

	LOG_EXT_TSK_ENTER();
	CHECK_TSKCTX();							/*［NGKI1164］*/

	if (sense_lock()) {
		/*
		 *  CPUロック状態でext_tskが呼ばれた場合は，CPUロックを解除し
		 *  てからタスクを終了する．実装上は，サービスコール内でのCPU
		 *  ロックを省略すればよいだけ．［NGKI1168］
		 */
	}
	else {
		lock_cpu();
	}

	intpri = t_get_ipm();
	if (!enadsp || intpri != TIPM_ENAALL) {
		if (!enadsp) {
			/*
			 *  ディスパッチ禁止状態でext_tskが呼ばれた場合は，ディスパッ
			 *  チ許可状態にしてからタスクを終了する．［NGKI1168］
			 */
			enadsp = true;
		}
		if (intpri != TIPM_ENAALL) {
			/*
			 *  割込み優先度マスク（IPM）がTIPM_ENAALL以外の状態で
			 *  ext_tskが呼ばれた場合は，IPMをTIPM_ENAALLにしてからタス
			 *  クを終了する．［NGKI1168］
			 */
			t_set_ipm(TIPM_ENAALL);
		}
		dspflg = true;
		p_schedtsk = search_schedtsk();
	}

#ifdef TOPPERS_SUPPORT_OVRHDR
	if (p_runtsk->staovr) {
		(void) target_ovrtimer_stop();
		ovrtimer_flag = false;
	}
#endif /* TOPPERS_SUPPORT_OVRHDR */
	task_terminate(p_runtsk);				/* ［NGKI3449］*/
	exit_and_dispatch();					/* ［NGKI1169］*/
	ercd = E_SYS;							/* ［NGKI1163］*/

  error_exit:
	LOG_EXT_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ext_tsk */

/*
 *  タスクの終了要求［NGKI3469］
 */
#ifdef TOPPERS_ras_ter

ER
ras_ter(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_RAS_TER_ENTER(tskid);
	CHECK_TSKCTX_UNL();							/*［NGKI3470］［NGKI3471］*/
	CHECK_ID(VALID_TSKID(tskid));				/*［NGKI3472］*/
	p_tcb = get_tcb(tskid);
	CHECK_ILUSE(p_tcb != p_runtsk);				/*［NGKI3475］*/

	lock_cpu();
	if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;							/*［NGKI3476］*/
	}
	else if (p_tcb->enater) {
		task_terminate(p_tcb);					/*［NGKI3477］*/
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	else {
		p_tcb->raster = true;					/*［NGKI3478］*/
		if (!TSTAT_RUNNABLE(p_tcb->tstat)) {
			if (TSTAT_WAITING(p_tcb->tstat)) {
				wait_dequeue_wobj(p_tcb);		/*［NGKI3479］*/
				wait_dequeue_tmevtb(p_tcb);
				p_tcb->p_winfo->wercd = E_RASTER;	/*［NGKI3480］*/
			}
			p_tcb->tstat = TS_RUNNABLE;			/*［NGKI3606］*/
			LOG_TSKSTAT(p_tcb);
			make_runnable(p_tcb);
			if (p_runtsk != p_schedtsk) {
				dispatch();
			}
		}
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_RAS_TER_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ras_ter */

/*
 *  タスク終了の禁止［NGKI3482］
 */
#ifdef TOPPERS_dis_ter

ER
dis_ter(void)
{
	ER		ercd;

	LOG_DIS_TER_ENTER();
	CHECK_TSKCTX_UNL();							/*［NGKI3483］［NGKI3484］*/

	lock_cpu();
	p_runtsk->enater = false;					/*［NGKI3486］*/
	ercd = E_OK;
	unlock_cpu();

  error_exit:
	LOG_DIS_TER_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_dis_ter */

/*
 *  タスク終了の許可［NGKI3487］
 */
#ifdef TOPPERS_ena_ter

ER
ena_ter(void)
{
	ER		ercd;

	LOG_ENA_TER_ENTER();
	CHECK_TSKCTX_UNL();							/*［NGKI3488］［NGKI3489］*/

	lock_cpu();
	if (p_runtsk->raster && dspflg) {
#ifdef TOPPERS_SUPPORT_OVRHDR
		if (p_runtsk->staovr) {
			(void) target_ovrtimer_stop();
			ovrtimer_flag = false;
		}
#endif /* TOPPERS_SUPPORT_OVRHDR */
		task_terminate(p_runtsk);
		exit_and_dispatch();
		ercd = E_SYS;
	}
	else {
		p_runtsk->enater = true;				/*［NGKI3491］*/
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_ENA_TER_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ena_ter */

/*
 *  タスク終了禁止状態の参照［NGKI3494］
 */
#ifdef TOPPERS_sns_ter

bool_t
sns_ter(void)
{
	bool_t	state;

	LOG_SNS_TER_ENTER();
	/*
	 *  enaterを変更できるのは自タスクのみであるため，排他制御せずに読
	 *  んでも問題ない．
	 */
	state = (p_runtsk != NULL && p_runtsk->enater) ? false : true;
												/*［NGKI3495］［NGKI3496］*/
	LOG_SNS_TER_LEAVE(state);
	return(state);
}

#endif /* TOPPERS_sns_ter */

/*
 *  タスクの強制終了［NGKI1170］
 */
#ifdef TOPPERS_ter_tsk

ER
ter_tsk(ID tskid)
{
	TCB		*p_tcb;
	ER		ercd;

	LOG_TER_TSK_ENTER(tskid);
	CHECK_TSKCTX_UNL();							/*［NGKI1171］［NGKI1172］*/
	CHECK_ID(VALID_TSKID(tskid));				/*［NGKI1173］*/
	p_tcb = get_tcb(tskid);
	CHECK_ILUSE(p_tcb != p_runtsk);				/*［NGKI1176］*/

	lock_cpu();
	if (TSTAT_DORMANT(p_tcb->tstat)) {
		ercd = E_OBJ;							/*［NGKI1177］*/
	}
	else {
		task_terminate(p_tcb);					/*［NGKI3450］*/
		if (p_runtsk != p_schedtsk) {
			dispatch();
		}
		ercd = E_OK;
	}
	unlock_cpu();

  error_exit:
	LOG_TER_TSK_LEAVE(ercd);
	return(ercd);
}

#endif /* TOPPERS_ter_tsk */
